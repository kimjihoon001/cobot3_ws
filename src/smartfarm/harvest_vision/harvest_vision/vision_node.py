"""원거리 탐지 + 근거리 품질 판정의 2단계 토마토 비전 노드."""

from __future__ import annotations

import os
from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from smartfarm_interfaces.msg import TomatoDetection, TomatoDetectionArray


IMAGE_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)
QUALITY_CLASSES = {"ripe", "spoiled"}


def _model_names(model) -> list[str]:
    names = model.names
    if isinstance(names, dict):
        return [str(names[index]) for index in sorted(names)]
    return [str(name) for name in names]


class VisionNode(Node):
    """토마토를 먼저 찾고, 가까운 목표 하나만 ripe/spoiled로 판정한다."""

    def __init__(self):
        super().__init__("vision_node")

        share = get_package_share_directory("harvest_vision")
        # 모두 상대 이름: 노드 namespace 아래에만 생성되어 다른 로봇과 섞이지 않는다.
        # namespace 없이 단독 실행하면 기존과 동일하게 /harvester/*, /vision/*가 된다.
        self.declare_parameter("rgb_topic", "harvester/rgb")
        self.declare_parameter("depth_topic", "harvester/depth")
        self.declare_parameter("camera_info_topic", "harvester/camera_info")
        self.declare_parameter("annotated_topic", "vision/annotated_image")
        self.declare_parameter("target_topic", "vision/approach_target")
        self.declare_parameter("target_class_topic", "vision/target_class")
        self.declare_parameter("detections_topic", "vision/tomato_detections")
        self.declare_parameter("detector_model_path", os.path.join(share, "finetuned_far.pt"))
        self.declare_parameter("quality_model_path", os.path.join(share, "finetuned_near.pt"))
        # 0.25→0.45: 잎사귀 오탐 억제(val P 0.77→0.96, 오탐 4.5→0.5/img). 수확은 여러
        # 프레임 투표라 재현율(0.80→0.59)보다 정밀도가 중요. (2026-07-23 threshold 스윕 근거)
        self.declare_parameter("detector_confidence", 0.45)
        self.declare_parameter("quality_confidence", 0.55)
        self.declare_parameter("near_distance_m", 0.50)
        self.declare_parameter("far_resume_distance_m", 0.60)
        self.declare_parameter("near_box_ratio", 0.025)
        self.declare_parameter("crop_padding_ratio", 0.20)
        self.declare_parameter("quality_vote_frames", 5)
        # 근거리 품질(ripe/spoiled) 모델 사용 여부. **기본 false = 원거리(tomato) 탐지만**
        # 쓰고 near 스테이지를 건너뛴다 → control_class 가 계속 "tomato" 라 파지로 바로 이어짐.
        # (2026-07-22 사용자: near 모델 검출 실패 블로커 회피 — far-only 를 기본으로.)
        # 근거리 품질판정을 다시 켜려면 param use_quality_model:=true.
        self.declare_parameter("use_quality_model", False)

        self._bridge = CvBridge()
        self._latest_depth: Image | None = None
        self._camera_info: CameraInfo | None = None
        self._quality_votes: deque[tuple[str, float]] = deque(
            maxlen=int(self.get_parameter("quality_vote_frames").value)
        )
        self._near_latched = False
        self._use_quality = bool(self.get_parameter("use_quality_model").value)

        self._detector = self._load_model("detector_model_path", expected={"tomato"})
        self._quality_model = (
            self._load_model("quality_model_path", expected=QUALITY_CLASSES)
            if self._use_quality else None
        )

        rgb_topic = str(self.get_parameter("rgb_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        self.create_subscription(Image, rgb_topic, self._rgb_callback, IMAGE_QOS)
        self.create_subscription(Image, depth_topic, self._depth_callback, IMAGE_QOS)
        self.create_subscription(CameraInfo, info_topic, self._camera_info_callback, IMAGE_QOS)
        self._detections_pub = self.create_publisher(
            TomatoDetectionArray,
            str(self.get_parameter("detections_topic").value),
            10,
        )
        self._annotated_pub = self.create_publisher(
            Image, str(self.get_parameter("annotated_topic").value), IMAGE_QOS
        )
        self._target_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("target_topic").value), 10
        )
        self._target_class_pub = self.create_publisher(
            String, str(self.get_parameter("target_class_topic").value), 10
        )

        self.get_logger().info(
            "2단계 비전 시작: 원거리=tomato 탐지, 근거리=ripe/spoiled 판정"
            if self._use_quality else
            "비전 시작: 원거리(tomato) 탐지만 사용 — 근거리 품질 스테이지 OFF"
        )

    def _load_model(self, parameter: str, expected: set[str]):
        path = os.path.expanduser(str(self.get_parameter(parameter).value))
        if not os.path.isfile(path):
            raise RuntimeError(f"{parameter} 파일이 없습니다: {path}")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics를 설치해야 vision_node를 실행할 수 있습니다"
            ) from exc

        model = YOLO(path)
        names = set(_model_names(model))
        if names != expected:
            raise RuntimeError(
                f"{parameter} 클래스가 잘못되었습니다: {sorted(names)} "
                f"(필요: {sorted(expected)})"
            )
        self.get_logger().info(f"{parameter}: {path} / classes={sorted(names)}")
        return model

    def _depth_callback(self, msg: Image):
        self._latest_depth = msg

    def _camera_info_callback(self, msg: CameraInfo):
        self._camera_info = msg

    def _rgb_callback(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        depth = self._depth_array(self._latest_depth)
        detections, annotated, target_pose, target_class = self._run_detection(
            frame, depth, msg
        )

        array_msg = TomatoDetectionArray()
        array_msg.header = msg.header
        array_msg.detections = detections
        # 빈 배열도 발행해야 FSM이 "현재 목표 없음"을 알 수 있다.
        self._detections_pub.publish(array_msg)
        # 일부 ROS Humble cv_bridge/OpenCV 조합은 CV_8UC3을 잘못된 키(16)로
        # 조회해 KeyError를 낸다. 출력 메시지는 단순 bgr8이므로 직접 구성한다.
        annotated_msg = self._bgr_image_message(annotated)
        annotated_msg.header = msg.header
        self._annotated_pub.publish(annotated_msg)
        if target_pose is not None:
            self._target_pub.publish(target_pose)
        # 빈 문자열도 발행해야 접근 제어기가 검출 소실을 즉시 알 수 있다.
        self._target_class_pub.publish(String(data=target_class or ""))

    @staticmethod
    def _bgr_image_message(frame: np.ndarray) -> Image:
        image = np.asarray(frame)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"annotated image는 BGR 3채널이어야 합니다: {image.shape}")
        if image.dtype != np.uint8:
            image = np.clip(image, 0, 255).astype(np.uint8)
        image = np.ascontiguousarray(image)
        msg = Image()
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(image.shape[1] * 3)
        msg.data = image.tobytes()
        return msg

    def _depth_array(self, msg: Image | None) -> np.ndarray | None:
        if msg is None:
            return None
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth = np.asarray(depth, dtype=np.float32)
        # ROS depth의 16UC1은 mm, 32FC1은 m가 관례다.
        if msg.encoding.upper() in {"16UC1", "MONO16"}:
            depth *= 0.001
        depth[~np.isfinite(depth)] = 0.0
        return depth

    def _run_detection(
        self, frame: np.ndarray, depth: np.ndarray | None, source_msg: Image
    ) -> tuple[list[TomatoDetection], np.ndarray, PoseStamped | None, str | None]:
        conf = float(self.get_parameter("detector_confidence").value)
        result = self._detector.predict(frame, conf=conf, verbose=False)[0]
        candidates = []
        for box in result.boxes:
            x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
            distance, pixel = self._tomato_measurement(
                frame, depth, x1, y1, x2, y2)
            candidates.append(
                {
                    "box": (x1, y1, x2, y2),
                    "confidence": float(box.conf[0]),
                    "distance": distance,
                    "pixel": pixel,
                }
            )

        if not candidates:
            self._quality_votes.clear()
            self._near_latched = False
            annotated = frame.copy()
            cv2.putText(
                annotated, "NO TOMATO", (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 0, 255), 2, cv2.LINE_AA,
            )
            return [], annotated, None, None

        # 깊이가 있으면 가장 가까운 토마토, 없으면 가장 큰 박스를 목표로 삼는다.
        with_depth = [item for item in candidates if item["distance"] is not None]
        target = (
            min(with_depth, key=lambda item: item["distance"])
            if with_depth
            else max(candidates, key=lambda item: self._box_area(item["box"]))
        )
        target_class, target_conf = "tomato", target["confidence"]
        near = self._is_near(target, frame.shape)
        if near:
            quality = self._classify_quality(frame, target["box"])
            if quality is not None:
                self._quality_votes.append(quality)
                stable = self._stable_quality()
                if stable is not None:
                    target_class, target_conf = stable
            else:
                self._quality_votes.clear()
        else:
            self._quality_votes.clear()

        messages = []
        for candidate in candidates:
            tomato_class = target_class if candidate is target else "tomato"
            confidence = target_conf if candidate is target else candidate["confidence"]
            messages.append(
                self._to_message(candidate, tomato_class, confidence, source_msg)
            )
        annotated = self._draw_detections(frame, candidates, target, messages)
        target_pose = self._pose_for_candidate(target, source_msg)
        # 근거리 진입 즉시 팔을 멈추고, 정지 상태에서 품질 투표를 끝낸다.
        control_class = (
            target_class if target_class in QUALITY_CLASSES else
            "quality_check" if near else "tomato"
        )
        return messages, annotated, target_pose, control_class

    def _is_near(self, candidate: dict, shape: tuple[int, ...]) -> bool:
        if not self._use_quality:      # 근거리 품질 스테이지 비활성 → 항상 far(tomato)
            self._near_latched = False
            return False
        distance = candidate["distance"]
        if distance is not None:
            threshold = (
                float(self.get_parameter("far_resume_distance_m").value)
                if self._near_latched
                else float(self.get_parameter("near_distance_m").value)
            )
            self._near_latched = distance <= threshold
            return self._near_latched
        image_area = float(shape[0] * shape[1])
        ratio = self._box_area(candidate["box"]) / image_area
        self._near_latched = ratio >= float(
            self.get_parameter("near_box_ratio").value
        )
        return self._near_latched

    def _classify_quality(
        self, frame: np.ndarray, box: tuple[float, float, float, float]
    ) -> tuple[str, float] | None:
        x1, y1, x2, y2 = self._padded_box(box, frame.shape)
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return None
        conf = float(self.get_parameter("quality_confidence").value)
        result = self._quality_model.predict(crop, conf=conf, verbose=False)[0]
        if len(result.boxes):
            best = max(result.boxes, key=lambda item: float(item.conf[0]))
            name = _model_names(self._quality_model)[int(best.cls[0])]
            return name, float(best.conf[0])

        # 근거리 detect 모델은 전체 카메라 프레임으로 학습됐을 수 있다. 타이트한 crop을
        # 640 입력으로 확대하면 과실 일부가 잘려 검출이 0개가 되는 경우가 있으므로,
        # 전체 프레임에서 다시 추론하고 원거리 타겟 중심 안에 든 품질 박스만 채택한다.
        full = self._quality_model.predict(frame, conf=conf, verbose=False)[0]
        matches = []
        tx1, ty1, tx2, ty2 = box
        for candidate in full.boxes:
            qx1, qy1, qx2, qy2 = (
                float(value) for value in candidate.xyxy[0].tolist()
            )
            cx, cy = (qx1 + qx2) * 0.5, (qy1 + qy2) * 0.5
            if tx1 <= cx <= tx2 and ty1 <= cy <= ty2:
                matches.append(candidate)
        if not matches:
            self.get_logger().warning(
                "근거리 품질 모델 검출 없음(crop/full-frame)",
                throttle_duration_sec=2.0,
            )
            return None
        best = max(matches, key=lambda item: float(item.conf[0]))
        name = _model_names(self._quality_model)[int(best.cls[0])]
        return name, float(best.conf[0])

    def _stable_quality(self) -> tuple[str, float] | None:
        required = self._quality_votes.maxlen
        if len(self._quality_votes) < required:
            return None
        name, count = Counter(item[0] for item in self._quality_votes).most_common(1)[0]
        if count <= required // 2:
            return None
        confidences = [confidence for label, confidence in self._quality_votes if label == name]
        return name, float(np.mean(confidences))

    def _median_depth(self, depth, x1, y1, x2, y2) -> float | None:
        if depth is None:
            return None
        height, width = depth.shape[:2]
        # 박스 중앙부만 사용해 배경 깊이가 섞이는 것을 줄인다.
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        half_w, half_h = max((x2 - x1) * 0.15, 1), max((y2 - y1) * 0.15, 1)
        xa, xb = max(int(cx - half_w), 0), min(int(cx + half_w) + 1, width)
        ya, yb = max(int(cy - half_h), 0), min(int(cy + half_h) + 1, height)
        values = depth[ya:yb, xa:xb]
        values = values[values > 0]
        return float(np.median(values)) if values.size else None

    def _tomato_measurement(self, frame, depth, x1, y1, x2, y2):
        """잎이 겹친 bbox에서도 과실 색 픽셀로 깊이와 투영 중심을 구한다.

        bbox 중심 깊이를 그대로 쓰면 앞쪽 잎이 토마토보다 10~30 cm 가까운 값을
        차지한다. 수확 대상인 익은 과실의 red/orange 고채도 픽셀만 골라 3D 점을
        만들고, 유효 픽셀이 부족할 때만 기존 중앙 ROI 방식으로 후퇴한다.
        """
        if depth is None:
            return None, ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
        height, width = depth.shape[:2]
        xa, xb = max(int(x1), 0), min(int(x2) + 1, width)
        ya, yb = max(int(y1), 0), min(int(y2) + 1, height)
        if xa >= xb or ya >= yb:
            return None, ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
        crop = frame[ya:yb, xa:xb]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        # OpenCV hue 0..179: 익은 토마토의 red/orange와 암적색 양쪽 범위.
        warm = (((hsv[..., 0] <= 25) | (hsv[..., 0] >= 165))
                & (hsv[..., 1] >= 70) & (hsv[..., 2] >= 45))
        local_depth = depth[ya:yb, xa:xb]
        valid = warm & np.isfinite(local_depth) & (local_depth > 0.0)
        ys, xs = np.nonzero(valid)
        if xs.size >= 12:
            values = local_depth[valid]
            return (float(np.median(values)),
                    (float(xa + np.median(xs)), float(ya + np.median(ys))))
        return (self._median_depth(depth, x1, y1, x2, y2),
                ((x1 + x2) * 0.5, (y1 + y2) * 0.5))

    def _to_message(self, item, tomato_class, confidence, source_msg):
        msg = TomatoDetection()
        msg.tomato_class = tomato_class
        msg.confidence = float(confidence)
        msg.pose = self._pose_for_candidate(item, source_msg) or PoseStamped()
        if not msg.pose.header.frame_id:
            msg.pose.header = source_msg.header
            msg.pose.pose.orientation.w = 1.0
        return msg

    def _pose_for_candidate(self, item, source_msg) -> PoseStamped | None:
        distance = item["distance"]
        info = self._camera_info
        if distance is None or info is None or not info.k[0] or not info.k[4]:
            return None
        u, v = item.get("pixel", (
            (item["box"][0] + item["box"][2]) * 0.5,
            (item["box"][1] + item["box"][3]) * 0.5,
        ))
        pose = PoseStamped()
        pose.header = source_msg.header
        pose.pose.position.x = (u - info.k[2]) * distance / info.k[0]
        pose.pose.position.y = (v - info.k[5]) * distance / info.k[4]
        pose.pose.position.z = distance
        pose.pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _draw_detections(frame, candidates, target, messages) -> np.ndarray:
        annotated = frame.copy()
        for candidate, message in zip(candidates, messages):
            x1, y1, x2, y2 = (int(value) for value in candidate["box"])
            selected = candidate is target
            color = (0, 255, 255) if selected else (0, 200, 0)
            if message.tomato_class == "ripe":
                color = (0, 255, 0)
            elif message.tomato_class == "spoiled":
                color = (0, 0, 255)
            thickness = 3 if selected else 2
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
            distance = candidate["distance"]
            distance_text = f" {distance:.2f}m" if distance is not None else ""
            prefix = "TARGET " if selected else ""
            label = (
                f"{prefix}{message.tomato_class} "
                f"{message.confidence:.2f}{distance_text}"
            )
            cv2.putText(
                annotated, label, (x1, max(y1 - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA,
            )
            if selected:
                center = ((x1 + x2) // 2, (y1 + y2) // 2)
                cv2.drawMarker(
                    annotated, center, color, cv2.MARKER_CROSS, 24, 2,
                )
        return annotated

    @staticmethod
    def _box_area(box) -> float:
        return max(box[2] - box[0], 0.0) * max(box[3] - box[1], 0.0)

    def _padded_box(self, box, shape) -> tuple[int, int, int, int]:
        x1, y1, x2, y2 = box
        padding = float(self.get_parameter("crop_padding_ratio").value)
        pad_x, pad_y = (x2 - x1) * padding, (y2 - y1) * padding
        height, width = shape[:2]
        return (
            max(int(x1 - pad_x), 0),
            max(int(y1 - pad_y), 0),
            min(int(x2 + pad_x), width),
            min(int(y2 + pad_y), height),
        )


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

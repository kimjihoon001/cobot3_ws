/** ui_status_node가 /ui/status로 보내는 JSON 모양. */
export interface RobotState {
  state: string;
  x: number | null;
  y: number | null;
  yaw: number | null;
}

export interface UiStatus {
  sim_time: number;
  cameras: Record<string, number>;
  robots: Record<string, RobotState>;
  events: { t: number; src: string; text: string }[];
}

export type CameraStatus = "online" | "warning" | "offline" | "error";
export type CameraType =
  | "robot_front"
  | "greenhouse"
  | "unloading"
  | "storage"
  | "overview";

/** <img>가 MJPEG를 실제로 받고 있는지. hz와 달리 브라우저 쪽 사정이다. */
export type StreamState = "loading" | "playing" | "error" | "reconnecting";

export interface CameraSpec {
  id: string;
  name: string;
  type: CameraType;
  /** /ui/<key>/compressed 및 status.cameras의 키. */
  key: string;
  /** status.robots의 키. 아직 연결 안 된 로봇이면 undefined. */
  robot?: string;
  /** 2×2 격자에 상시 표시할지. 조감도는 확대했을 때만 띄운다. */
  inGrid: boolean;
}

/**
 * 왼쪽 위에서 오른쪽 아래로 공정이 이어지도록 배치한다.
 * 검출 → 수확·전달 → 인계 → 적재.
 */
export const CAMERAS: CameraSpec[] = [
  { id: "CAM-01", name: "MM D455 / YOLO", type: "robot_front", key: "mm_front", robot: "mm", inGrid: true },
  { id: "CAM-02", name: "GREENHOUSE OPERATION", type: "greenhouse", key: "greenhouse", robot: "mm", inGrid: true },
  { id: "CAM-03", name: "UNLOADING AREA", type: "unloading", key: "unloading", robot: "iw", inGrid: true },
  { id: "CAM-04", name: "WAREHOUSE STORAGE", type: "storage", key: "storage", robot: "forklift", inGrid: true },
  { id: "CAM-05", name: "OVERVIEW", type: "overview", key: "overview", inGrid: false },
];

/**
 * 카메라 건강 상태. fps는 ui_status_node가 압축 토픽 도착 간격으로 실측한
 * 값이라 지어낸 숫자가 아니다. 스트림이 끊긴 건 브라우저만 알 수 있어서
 * 두 신호를 같이 본다.
 */
export function cameraHealth(hz: number, stream: StreamState): CameraStatus {
  if (stream === "error" || stream === "reconnecting") return "error";
  if (hz <= 0) return "offline";
  if (hz < 10) return "warning";
  return "online";
}

export const VIDEO_HOST = `http://${location.hostname}:8080`;
export const ROSBRIDGE_URL = `ws://${location.hostname}:9090`;

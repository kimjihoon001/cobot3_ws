# Isaac Sim(Humble, domain 108)과 이 워크스테이션(Jazzy, domain 109) 사이를 중계하는
# domain_bridge 전용 컨테이너. Humble로 빌드하는 이유는 main.py/isaacpjt에서 확인한
# 크래시(2026-07-27) 때문 — domain_bridge 프로세스가 링크하는 rmw는 양쪽 도메인에
# 공통으로 쓰이는데, Jazzy Fast-DDS(2.14.x)로 빌드하면 108쪽 참가자도 Jazzy가 돼서
# Isaac 내장 Humble Fast-DDS(2.6.10)가 ros_discovery_info를 잘못 역직렬화해 죽는
# 문제가 그대로 재현된다. Humble로 빌드해야 108쪽 참가자가 Isaac과 버전이 맞는다.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-domain-bridge \
    && rm -rf /var/lib/apt/lists/*

COPY config.yaml /config.yaml

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && ros2 run domain_bridge domain_bridge /config.yaml --ros-args --log-level debug"]

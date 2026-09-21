# 로컬(Mac colima) 및 Astra 검증용 — 협업자 머신(Ubuntu 22.04 + ROS 2 Humble + Python 3.10)과 동일 조건
FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-aiohttp python3-yaml python3-pytest python3-pygame python3-pip \
      ros-humble-nav2-msgs iputils-ping \
    && rm -rf /var/lib/apt/lists/*
# 저장소는 /ws 에 bind mount, 인터페이스 빌드 산출물은 /ws_build (named volume)
WORKDIR /ws
COPY entry.sh /entry.sh
RUN chmod +x /entry.sh
ENTRYPOINT ["/entry.sh"]
CMD ["bash"]

# Dockerfile
FROM osrf/ros:humble-desktop

# 기본 도구 및 의존성 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    libgl1-mesa-dev \
    libosmesa6-dev \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo 및 Meshcat 관련 파이썬 패키지 설치
RUN pip3 install \
    mujoco \
    mujoco-python-viewer \
    meshcat \
    numpy \
    transforms3d

# ROS2 워크스페이스 생성
WORKDIR /ros2_ws/src

# MuJoCo Menagerie 클론 (UR5e 모델 포함)
RUN git clone https://github.com/google-deepmind/mujoco_menagerie.git

# 환경 설정
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# 엔트리포인트 설정
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
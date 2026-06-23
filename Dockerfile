# EgoSteer robot stack: ROS 2 Humble + Python/system deps. Build:
#   docker build -t egosteerai/robot-stack:1.0.0 -t egosteerai/robot-stack:latest - < Dockerfile
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-dev build-essential cmake git wget unzip ca-certificates iproute2 \
      ros-humble-rmw-cyclonedds-cpp \
      libsdl2-2.0-0 libvulkan1 mesa-vulkan-drivers libcap2-bin xdg-utils libpipewire-0.3-0 \
      liblz4-dev libzstd-dev libre2-dev \
      fonts-noto-cjk mpg123 \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir -U pip wheel \
 && python3 -m pip install --no-cache-dir \
      "numpy<2" \
      mujoco==3.3.5 mink==0.0.12 \
      openvr==2.5.102 pyserial pyrealsense2 \
      opencv-python "scipy>=1.14.0" \
      websockets pynput requests msgpack \
      loop-rate-limiters protobuf pyzmq matplotlib python-can

# ruckig 0.14.0 builds from source; pin pybind11 (latest breaks its bindings).
RUN echo "pybind11==2.13.6" > /tmp/constraints.txt \
 && PIP_CONSTRAINT=/tmp/constraints.txt python3 -m pip install --no-cache-dir ruckig==0.14.0 \
 && rm -f /tmp/constraints.txt

# rerun-sdk (Python) for replay_rrd: reads .rrd via the dataframe API. Installed with --no-deps so
# it does not pull numpy>=2 — the dataframe read works fine with the numpy<2 main stack. attrs is
# upgraded (the apt python3-attr does not expose the importable `attrs` module rerun needs).
RUN python3 -m pip install --no-cache-dir --no-deps rerun-sdk==0.23.3 \
 && python3 -m pip install --no-cache-dir -U attrs pyarrow typing_extensions

# Rerun C++ SDK (for the record node): build and install to /opt/rerun_sdk_install.
# Arrow is built from source here, using lz4/zstd/re2 from the apt step above.
RUN cd /tmp \
 && wget -q https://github.com/rerun-io/rerun/releases/download/0.23.3/rerun_cpp_sdk.zip \
 && unzip -q rerun_cpp_sdk.zip \
 && cmake -S rerun_cpp_sdk -B rerun_build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/rerun_sdk_install \
 && cmake --build rerun_build -j"$(nproc)" \
 && cmake --install rerun_build \
 && rm -rf /tmp/rerun_cpp_sdk.zip /tmp/rerun_cpp_sdk /tmp/rerun_build

WORKDIR /root/workspace/robot-stack
CMD ["bash"]

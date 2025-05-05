FROM nvcr.io/nvidia/cuda:12.6.1-runtime-ubuntu22.04@sha256:aa0da342b530a7dd2630b17721ee907fc9210b4e159d67392c4c373bef642483

RUN apt-get update \
    && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends --allow-change-held-packages \
        libgtk-3-dev \
        libpython3.10 \
        libxkbcommon-x11-0 \
        python3-pip \
        unzip \
        vulkan-tools \
        mesa-vulkan-drivers \
        wget \
        ffmpeg \
    && apt-get clean

RUN pip3 install --upgrade pip

WORKDIR /pycuvslam

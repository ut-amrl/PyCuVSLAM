#!/bin/bash
docker run -it \
  --rm \
  --gpus all \
  --network host \
  --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=8 \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v .:/pycuvslam \
  pycuvslam-ros2

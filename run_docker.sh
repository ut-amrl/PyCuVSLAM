#!/bin/bash
XSOCK=/tmp/.X11-unix
XAUTH=~/pycuvslam/tmp/.docker.xauth
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH
docker run -it \
  --rm \
  --gpus all \
  --privileged \
  --network host \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=$XAUTH \
  -e XDG_RUNTIME_DIR=/run/user/$(id -u) \
  -v /run/user/$(id -u):/run/user/$(id -u) \
  -v $XSOCK:$XSOCK \
  -v $XAUTH:$XAUTH \
  -v .:/pycuvslam \
  -v /robodata/public_datasets/frodobots8k:/frodobots8k \
  -v ~/frodobots-pose-extractor:/frodobot-pose-extractor \
  pycuvslam
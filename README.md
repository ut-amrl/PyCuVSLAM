# Visual Wheel Odometry ROS2 Wrapper

## Installation

1. Build Docker
    ```bash
    docker build -f Dockerfile_ros2 . --network host --tag pycuvslam-ros2
    ```

2. Run Container
    ```bash
    ./run_docker_ros2.sh
    ```

3. Setup PyCuVSLAM and Build ros2_ws
    ```bash
    ./setup_vwo.sh
    ```

4. Run ROS2 node
    ```bash
    ros2 run pycuvslam_ros2 vwo_node
    ```
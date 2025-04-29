# PyCuVSLAM

PyCuVSLAM is the official Python wrapper around the cuVSLAM visual-inertial SLAM (Simultaneous Localization And Mapping)
software package developed by NVIDIA. It is a highly accurate and computationally efficient package
using CUDA acceleration for real-time visual-inertial SLAM.

![Demo](pycuvslam.gif)

## System Requirements

PyCuVSLAM is supported on the following OS and platforms, with the system requirements and installation methods listed below:

| OS                                | Architecture | System Requirements                          | Supported Installation Method                   |
|-----------------------------------|--------------|----------------------------------------------|-------------------------------------------------|
| Ubuntu 22.04 (Desktop/Laptop)     | x86_64       | Python 3.10, Nvidia GPU with CUDA 12.6       | [Native][3], [Venv][4], [Conda][5], [Docker][6] |
| Ubuntu 24.04 (Desktop/Laptop)     | x86_64       | Nvidia GPU with CUDA 12.6                    | [Conda][5], [Docker][6]                         |
| Ubuntu 22.04 ([Nvidia Jetson][1]) | aarch64      | [Jetpack 6.1/6.2][2], Python 3.10, CUDA 12.6 | [Native][3], [Venv][4], [Conda][5], [Docker][6] |

[1]: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/
[2]: https://docs.nvidia.com/jetson/archives/jetpack-archived/jetpack-62/
[3]: #option-1-native-install
[4]: #option-2-using-venv
[5]: #option-3-using-conda
[6]: #option-4-using-docker


### CUDA Toolkit
Make sure you have the CUDA Toolkit installed, you can download the toolkit from the [NVIDIA website](https://developer.nvidia.com/cuda-downloads). If you install the CUDA toolkit for the first time, make sure to restart your computer.

## Install Options
Depending on your OS and platform and the supported installation method, follow the instructions below to install PyCuVSLAM.

### Option 1: Native Install
**Important**: This option is only available for Ubuntu 22.04 x86_64 and Jetpack 6.1/6.2 aarch64.

There are no special instructions for native install, proceed to [PyCuVSLAM installation](#pycuvslam-installation).

### Option 2: Using Venv
**Important**: This option is only available for Ubuntu 22.04 x86_64 and Jetpack 6.1/6.2 aarch64.  

Create a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

When done using PyCuVSLAM, to deactivate the virtual environment, run `deactivate`.

Proceed to [PyCuVSLAM installation](#pycuvslam-installation).  

### Option 3: Using Conda

**Important**: This option has been tested on Ubuntu 22.04 x86_64 and Ubuntu 24.04 x86_64. It *may* work on other
versions of Ubuntu x86_64, but it has not been tested.  

Create a conda environment and install the required packages:

```bash
conda create -n pycuvslam python==3.10 pip
conda activate pycuvslam
conda install -c conda-forge libstdcxx-ng=12.2.0
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
```
**Important**: The `LD_LIBRARY_PATH` environment variable must be set every time you activate the
conda environment to ensure that the correct `libpython` library is loaded.  

To deactivate the conda environment when you are done using PyCuVSLAM, run `conda deactivate`.

Proceed to [PyCuVSLAM installation](#pycuvslam-installation).  

### Option 4: Using Docker
1. Setup ngc
    ```bash
    docker login nvcr.io --username '$oauthtoken'
    ```
    For password enter key for ngc catalog from: https://org.ngc.nvidia.com/setup/api-keys

2. Build docker image
    ```bash
    git clone https://github.com/NVlabs/pycuvslam.git
    cd pycuvslam
    docker build . --network host --tag pycuvslam
    ```

3. Run container from image, and install cuvslam
    ```bash
    ./run_docker.sh
    ```
To exit the container when you are done using the container, run `exit`.

Proceed to step 2 of [PyCuVSLAM installation](#pycuvslam-installation).

## PyCuVSLAM Installation

1. Clone the PyCuVSLAM repository.
    ```bash
    git clone https://github.com/NVlabs/pycuvslam.git
    cd pycuvslam
    ```

2. Install the PyCuVSLAM package.
    ```bash
    pip install -e cuvslam/x86
    ```
    For Jetson, use the following command:
    ```bash
    pip install -e cuvslam/aarch64
    ```

## Run Examples
1. Install PyCuVSLAM using one of the installation methods mentioned above, and then install the
   required packages for the examples:
    ```bash
    pip install -r examples/requirements.txt
    ```
2. Run the examples:
    - [EuRoC Dataset Tutorial](docs/tutorial_euroc.md)
    - [KITTI Dataset Tutorial](docs/tutorial_kitti.md)
    - [Multi-Camera Dataset Tutorial](docs/tutorial_multicamera_edex.md)
    - [OAK-D Camera Tutorial](docs/tutorial_oakd.md)
    - [RealSense Camera Tutorial](docs/tutorial_realsense.md)

## API Documentation

For detailed API documentation, please refer to the [API Documentation](https://nvlabs.github.io/PyCuVSLAM/api.html) page.

## Notes about SLAM performance
The accuracy and robustness of a SLAM system can be effected by many different factors, here is a small list of some of these factors:

- <strong>Intrinsics and extrinsics calibration</strong> accuracy are crucially important. Make sure your calibration is accurate. If you've never calibrated before, work with an experienced vendor to have your setup calibrated.
- <strong>Synchronization and time stamping</strong> will make a huge difference to the performance of cuVSLAM. Make sure that your multi-camera images are captured at the same exact time, preferably through hardware synchronization, and that the relative timestamps of the captured images are correct across different cameras.
- <strong>Frame rate</strong> can effect performance significantly. The ideal frame rate depends on your translational and rotational velocity. 30 FPS should work for most "human speed" movement.
- <strong>Resolution</strong> is important. Minimum of 720p is recommended. cuVSLAM can operate on relatively high resolution image streams efficiently due to its CUDA acceleration.
- <strong>Image quality</strong> is also very important. Make sure you have good lenses, exposure and white balancing that does not clip large parts of the image, and clean the dirt from your lenses. Motion blur can cause issues so make sure your exposure time is short enough to avoid motion blur. If your images are too dark, increase lighting in your environment.
- <strong>Compute</strong> can effect your SLAM performance. If cuVSLAM does not have enough compute to keep up with the image streams, performance might degrade.

## Issues

### Issues pip installing PyCuVSLAM

If you're having issues ```pip install```ing PyCuVSLAM because of lack of permission, try updating your pip:

```bash
python -m pip install -U pip
```

### Reporting other issues
Are you having problems running PyCuVSLAM? Do you have any suggestions? We'd love to hear your feedback in the [issues](https://github.com/NVlabs/pycuvslam/issues) tab.

## ROS2 Support

If you would like to use cuVSLAM in a ROS2 environment, please refer to the following links:
* [Isaac ROS cuVSLAM GitHub](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
* [Isaac ROS cuVSLAM Documentation](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html)
* [Isaac ROS cuVSLAM User Manual](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)


## License
This project is licensed under a non-commercial NVIDIA license, for details refer to the [LICENCE](LICENSE) file.


# ROS2 Wrapper

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
    ./setup_pycuvslam.sh
    ```

4. Run ROS2 node
    ```bash
    ros2 run pycuvslam_ros2 main_node
    ```
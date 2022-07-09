# k4a Projector

[VECtor Benchmark](https://star-datasets.github.io/vector/) is the first complete set of benchmark datasets captured with a multi-sensor setup containing an event-based stereo camera, a regular stereo camera, multiple depth sensors, and an inertial measurement unit. The setup is fully hardware-synchronized and underwent accurate extrinsic calibration. All sequences come with ground truth data captured by highly accurate external reference devices such as a motion capture system. Individual sequences include both small and large-scale environments, and cover the specific challenges targeted by dynamic vision sensors.

The k4a projector is a function that uses carefully-calibrated extrinsics to reproject depth readings from the Kinect depth camera onto other sensor frames. It is offered as an independent repository, other than the [MPL Calibration Toolbox](https://github.com/mgaoling/mpl_calibration_toolbox) and the [MPL Dataset Toolbox](https://github.com/mgaoling/mpl_dataset_toolbox), given its high dependency on the [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK).

# License and Citation

This toolbox, together with the [MPL Calibration Toolbox](https://github.com/mgaoling/mpl_calibration_toolbox) and the [MPL Dataset Toolbox](https://github.com/mgaoling/mpl_dataset_toolbox), is available as open-source under the terms of the [BSD-3-Clause-Clear License](https://github.com/greatoyster/k4a_projector/blob/main/LICENSE.txt). If you use this toolbox in an academic context, please cite the [publication](https://star-datasets.github.io/vector/assets/pdf/VECtor.pdf) as follows:

```bibtex
@Article{gao2022vector,
  author  = {Gao, Ling and Liang, Yuxuan and Yang, Jiaqi and Wu, Shaoxun and Wang, Chenyu and Chen, Jiaben and Kneip, Laurent},
  title   = {{VECtor}: A Versatile Event-Centric Benchmark for Multi-Sensor SLAM},
  journal = {IEEE Robotics and Automation Letters},
  pages   = {8217--8224},
  volume  = {7},
  number  = {3},
  year    = {2022},
  doi     = {10.1109/LRA.2022.3186770}
}
```

# Getting Started

The following instructions are tested on [Ubuntu 18.04](https://ubuntu.com/download/desktop) with [ROS Melodic](http://wiki.ros.org/ROS/Installation). A ROS **desktop-full installation** is required. On top of that, the following libraries ([Eigen 3](https://eigen.tuxfamily.org/index.php?title=Main_Page), [OpenCV](https://opencv.org/releases/), [yaml-cpp](https://github.com/jbeder/yaml-cpp)) have to be installed:

```
sudo apt-get update
sudo apt-get install libeigen3-dev libopencv-dev libyaml-cpp-dev
ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

Azure Kinect Sensor SDK (more details on [this page](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download#linux-installation-instructions)) is also required:

```
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
sudo apt-get update
sudo apt-get install k4a-tools
sudo apt install libk4a1.4-dev
```

After that, enter your catkin workspace and the build can be triggered with the following command:

```
cd ~/catkin_ws/src
git clone https://github.com/greatoyster/k4a_projector.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# Usage

This function is tailored for the [VECtor Benchmark](https://star-datasets.github.io/vector/), hence some of the information is hard-coded in the script. User should first check and modify the parameters in the `config/config.yaml`, then place the data sequence(s) and the related calibration result(s) into the `data` folder. Note that we have already offered four calibration results as examples which cover all the possible sensor frames to be projected. Launch the projector by:

```
roslaunch k4a_projector run.launch
```

**Note:** Unlike image data, event data (if included in the input) will **NOT** be undistorted given its sparse nature. However, every reprojected depth image, including the ones on event frames, will **BE** undistorted (which is enforced by the Azure Kinect Sensor SDK). User should be aware of this and perform the undistortion if necessary.

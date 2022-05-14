# k4a projector

Project azure kinect depth measurement to provided virtual frame with official driver for Ubuntu18.04 or other distribution with libk4a support

# Usage

The input is a rosbag contains depth images of uint16 format and virtual camera calibration data.

You just need to edit `./config/config.yaml` and `./config/target.yaml`

# How to install libk4a

Please refer to [azure kinect driver](https://docs.microsoft.com/zh-cn/azure/Kinect-dk/sensor-sdk-download) 

We use `libk4a1.4-dev` in this project.

```bash
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -

sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod

sudo apt-get update
```

# How to run example

We use `catkin_simple` as dependency, just put it in your catkin workspace `src` directory

Compile with `catkin_make`.

After editing configuration files, run with `roslaunch k4a_projector run.launch`


# cyclops_playground
Demonstration playground for [cyclops_ros](https://github.com/cyclops-double-blind/cyclops_ros)
on EuRoC-MAV dataset. This software repository is supplementary material for an
article we submitted for review to IEEE Transactions on Robotics in 2025. The
authors of this repository (license holders, commit authors, repository
organization) are anonymized for the double-blind review.

### See also:
* [cyclops](https://github.com/cyclops-double-blind/cyclops)
* [cyclops_ros](https://github.com/cyclops-double-blind/cyclops_ros)

## Prerequisites:
* **Docker**: follow
  [docker installation guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
  and [docker post-installation guide](https://docs.docker.com/engine/install/linux-postinstall/).
* **Docker SDK for python**: run `$ pip install docker`.
* **Glog**, **Gflags**: run (tested under Ubuntu 20.04)
  ``` bash
  $ sudo apt-get update && \
    sudo apt-get install -y libgflags-dev libgoogle-glog-dev
  ```

## Installation
The below instructions are tested under Ubuntu 20.04 and ROS noetic.

* Clone the repository:
  ``` bash
  $ git clone https://github.com/cyclops-double-blind/cyclops_playground \
    --recursive
  ```
* Build the workspace:
  ``` bash
  $ cd ./cyclops_playground/catkin_ws && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -Dcyclops_native_build=yes
  ```
* Source the workspace:
  ``` bash
  $ source devel/setup.bash
  ```
* Pull the EuRoC-MAV dataset:
  ``` bash
  $ `rospack find cyclops_launch_script`/tools/pull_dataset
  ```

## Interactive reset demonstration
The following commands each launches an interactive demonstration that runs
Cyclops and ORB-SLAM3 in EuRoC-MAV dataset, and re-initializes each VIO by user's
manual reset button clicks.
``` bash
$ roslaunch cyclops_launch_script main.launch dataset:=MH_01 auto_reset:=false mode:=cyclops
$ roslaunch cyclops_launch_script main.launch dataset:=MH_01 auto_reset:=false mode:=orbslam
```

## Replicating our paper's experiment result
To replicate the experiment result of our paper (with Python-generated random
re-initialization time points), omit the `auto_reset:=false` launch argument, and
record the telemetry topics.
``` bash
$ roslaunch cyclops_launch_script main.launch dataset:=MH_01 mode:=cyclops
$ rosbag record -o cyclops_result \
    /cyclops_backend/init/accept \
    /cyclops_backend/init/attempt \
    /cyclops_backend/init/failure \
    /cyclops_backend/init/success \
    /cyclops_backend/init/success/detail \
    /cyclops_backend/keyframe \
    /cyclops_backend/start \
    /reset \
```

For ORB-SLAM3, run:
``` bash
$ roslaunch cyclops_launch_script main.launch dataset:=MH_01 mode:=orbslam
```

The initialization result (success timestamp, input keyframes pose, scale
solution) for ORB-SLAM3 does not come out as ROS topics. Instead it is saved as a
log file, in the directory:
``` bash
$ roscd cyclops_launch_script/logs && ls && cd -
# orbslam3.dump.initialization  orbslam3.dump.lost  tallylog
```

# WiLD-Align

**WiFi Localization Driven Frame Alignment (WiLD-Align) is a frame alignment pipeline that incorporates Wi-Fi RSSI-based range measurements into inter-robot frame alignment. By narrowing the loop closure search space and prioritizing keyframes for refinement, WiLD-Align reduces perceptual aliasing, communication requirements, and extraneous point cloud registrations.**

<p align="center">
    <img src="./doc/Example.gif" alt="WiLD-Align Demo" width="800"/>
</p>

<p align="center">
    <table>
        <tr>
            <td><img src="./doc/BoWG.png" width="250"/></td>
            <td><img src="./doc/BoWN.png" width="250"/></td>
            <td><img src="./doc/BoWB.png" width="250"/></td>
        </tr>
        <tr>
            <td><img src="./doc/RangeG.png" width="250"/></td>
            <td><img src="./doc/RangeN.png" width="250"/></td>
            <td><img src="./doc/RangeB.png" width="250"/></td>
        </tr>
        <tr>
            <td><img src="./doc/WiLDG.png" width="250"/></td>
            <td><img src="./doc/WiLDN.png" width="250"/></td>
            <td><img src="./doc/WiLDB.png" width="250"/></td>
        </tr>
    </table>
</p>

*Top row:* Perceptual aliasing without WiLD-Align  
*Middle row:* Range alignments  
*Bottom row:* WiLD-Align results  
(Columns: Garage, Neighborhood, Building)

---

## Menu

- [**System Architecture**](#system-architecture)
- [**Package Dependencies**](#package-dependencies)
- [**Installation & Launch**](#installation)
- [**Setup**](#setup)
- [**Datasets**](#datasets)
- [**TODO**](#todo)
- [**Acknowledgement**](#acknowledgement)

---

## System Architecture

WiLD-Align integrates range measurements from Wi-Fi RSSI into the inter-robot frame alignment process, informing the keyframe search. The range-based model of another robot's trajectory creates a strong starting point for a bag of words (BoW) selection of keyframes. Only these keyframes are exchanged and used in registration to generate relative measurements. Finally, they are put through Pairwise Consistency Maximization (PCM) to identify mutully consistent measurements for a tight trajectory alignment.

<p align="center">
    <img src="./doc/architecture.png" alt="WiLD-Align Architecture" width="800"/>
</p>

---

## Package Dependencies
- **[ROS Noetic](http://wiki.ros.org/noetic)**  
  This package was developed and tested using ROS Noetic on Ubuntu 20.04 and has not yet been validated on other ROS distributions.

- **[GTSAM](https://gtsam.org/) (>= 4.0.3)**  
  GTSAM is used to support Pairwise Consistency Maximization (PCM) and to define keyframe and pose-graph–related functionality.  
  One supported installation method is via the official BorgLab PPA:
  ```bash
  sudo add-apt-repository ppa:borglab/gtsam-release-4.0
  sudo apt install libgtsam-dev
  
- **[PCL](https://pointclouds.org/)**
  PCL is used for point cloud processing in WiLD-Align. PCL is typically installed automatically as part of a standard ROS Noetic installation
  
- **[Eigen](https://gitlab.com/libeigen/eigen/-/releases/3.3.7)** 
  Eigen is used for many of our operations and is installed by default with ROS Noetic.
---

## Installation & Launch

### Install & Launch WiLD-Align
This project uses a fork of [LIO-SAM](https://github.com/AnonymousUser964/LIO-SAM), which requires the arguement `--recurse-submodules` to be included during cloning. The following allows you to clone, build, and launch a typical instance of WiLD-Align:

```bash
mkdir ~/wild_align_ws
cd ~/wild_align_ws
git clone --recurse-submodules https://github.com/AnonymousUser964/WiLD_Align.git
cd WiLD_Align/scripts
bash build_all.sh
bash launch.sh
```

### Launch WiLD-Align Demo Mode
To test the project with the included rosbags, you will need to switch to the demo branch which includes additional build/launch files for creating a second instance of WiLD-Align, and a communication node to facilitate communication between the instances.

```bash
cd ~/wild_align_ws/WiLD_Align
git checkout demo
cd scripts
bash build_all_test.sh
bash launch_test.sh
```

## Setup

### General
There are configurations you will want to set for your specific system before running WiLD-Align, found in **`src/wild_align/src/WiLD_Align/config/params.yaml`**. These should be adjusted based on the quality of your incoming data, the maximum reliable distance to generate loop closures over, the accuracy of your ranging device (which is WiFi RSSI in our paper), and the minimum number of measurements you want to consider before attempting to integrate range-based alignments

- **registrationSearchDistance:** The max distance between range-aligned trajectories to be considered for loop closures; indoor/confined-->5-10m, outdoor/open-->15-25m
- **rangeNoiseBound:** The max range where distance measurements become dominated by noise; RSSI indoor/confined-->5-10m, RSSI outdoor/open-->15-30m

- **use2Dmapping:** Switch between 3D (good for multi-level environments) and 2D (faster and good for cartesian environements) GNC_LM range solvers

- **trustRSSIThreshold:** Sets a minimum range measurement requirement before solving (set to no less than 3)

- **residualErrorThresh:** Sets ICP fitness threshold for loop closure calculations (will depend on the quality of point clouds)

- **bowSearchWindow:** Sets pose neighborhood for the Bag of Words search of possible loop closures 

### Demo 
There are additional configurations used for demo-ing, that are designed for visualizations or post-test analysis. Within the demo branch, these params are set along with the above to provide visualizations of the alignments achieved.

- **demoMode:** When run in the 2 robot demo format, allows for the complete and seperate exchange of keyframes for RVIS visualization only. When set to false, only keyframes properly exchanged are presented in RVIS.

- **reportData:** When outputing the alignment pcd file, this will output the volume of data received per robot and the number of point cloud registrations required to achieve the given alignment.  


### Adding your own Lidar SLAM system 
> **Note:** This project includes a fork of LIO-SAM, whose parameters are adjusted to match the topics and data in the bags we provide. 
1. **Edit `src/wild_align/src/WiLD_Align/config/params.yaml`** to reflect your desired keyframe and ranging topics, and any expected thresholds.
2. **Add your SLAM system to src/slam/src** and remove LIO-SAM.
3. **Add the following ROS publisher and keyframe/trajectory publisher** to your SLAM system:
```cpp
#include <wildalign_msgs/keyframe.h>

ros::Publisher pubKeyframe;
pubKeyframe = nh.advertise<wildalign_msgs::keyframe>("wild_align/local_keyframe", 1);

static int trajectoryIter = 0;
static int numberMessagesSent = 0;
wildalign_msgs::keyframe thisKeyframe;
thisKeyframe.header = YourHeaderHere;

if (HasAnUpdateHappened) {
    std::vector<geometry_msgs::Pose> trajectory;
    /* Example for using iSAM2 estimate
    for (int i = 0; i < isamEstimate.size(); i++) {
        geometry_msgs::Pose pose;
        pose.position.x = isamEstimate.at<Pose3>(i).translation().x();
        pose.position.y = isamEstimate.at<Pose3>(i).translation().y();
        pose.position.z = isamEstimate.at<Pose3>(i).translation().z();
        pose.orientation.x = isamEstimate.at<Pose3>(i).rotation().roll();
        pose.orientation.y = isamEstimate.at<Pose3>(i).rotation().pitch();
        pose.orientation.z = isamEstimate.at<Pose3>(i).rotation().yaw();
        trajectory.push_back(pose);
    }
    */
    thisKeyframe.trajectoryIter = trajectoryIter++;
    thisKeyframe.poses = trajectory;
}

interPose.header = cloudInfo.header;
interPose.header.seq = numberMessagesSent;
interPose.pose.position.x = CurrentPose.x;
interPose.pose.position.y = CurrentPose.y;
interPose.pose.position.z = CurrentPose.z;
interPose.pose.orientation.x = CurrentPose.roll;
interPose.pose.orientation.y = CurrentPose.pitch;
interPose.pose.orientation.z = CurrentPose.yaw;
thisKeyframe.pose = interPose.pose;
thisKeyframe.cloud = CurrentKeyframeCloud;
thisKeyframe.seq = numberMessagesSent;
thisKeyframe.purpose = 2;
thisKeyframe.identifier = 0;
pubKeyframe.publish(thisKeyframe);
numberMessagesSent++;
```
4. **Add to `CMakeLists.txt`** under `find_package`, `generate_messages`, and `catkin_package`:
```
wildalign_msgs
```
5. **Add to `package.xml`**:
```xml
<build_depend>wildalign_msgs</build_depend> 
<run_depend>wildalign_msgs</run_depend> 
```
6. **Recompile** either through catkin_make or with scripts/build_all.sh

## Datasets
Robot pair datasets can be found on [Google Drive](https://drive.google.com/drive/folders/1nBQmlLJm1U56DQsAWjlI_VHNSGsOqWHh?usp=sharing) — containing LiDAR, IMU, and RSSI captures from two robots.These are designed to be run with the demo branch of this project.

To run a dataset, you will download a zip file of your selection which contains 1 rosbag and 2 robot.yaml param files. Our experiement Jackal robots unfortunally have different quality lidars and components leading to different param requirements for each one during an experiment. After switing your wild-align to the demo branch, you will need to insert the robot.yaml files into **`src/wild_align/src/WiLD_Align/config`**, replacing robot1.yaml and robot2.yaml. You can then launch the project and run the attached bag.     
 

## TODO
- Update to ROS2 and provide CSLAM system integrations  
- Consolidate additional variables into `params.yaml`  

## Acknowledgement
- WiLD-Align’s ROS framework was inspired by **[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)**  
- WiLD-Align uses to complete its BoW searches **[Scan Context](https://github.com/irapkaist/scancontext)**  
- WiLD-Align uses to complete outlier rejection of inconsistent relative measurements **[Pairwise Consistency Maximization (PCM)](https://github.com/ethz-asl/pcm)**
- WiLD-Align uses to generate its range-based trajectory model **[Pairwise Maximum Clique (PMC) Library](https://github.com/ryanrossi/pmc)** 
```



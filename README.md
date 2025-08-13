# WiLD-Align

**WiFi Localization Driven Frame Alignment (WiLD-Align) is a frame alignment pipeline that incorporates Wi-Fi RSSI-based range measurements into inter-robot frame alignment. By narrowing the loop closure search space and prioritizing keyframes for refinement, WiLD-Align reduces perceptual aliasing, communication requirements, and extraneous point cloud registrations.**

<p align="center">
    <img src="./config/doc/Example.gif" alt="WiLD-Align Demo" width="800"/>
</p>

<p align="center">
    <table>
        <tr>
            <td><img src="./config/doc/BoWG.png" width="250"/></td>
            <td><img src="./config/doc/BoWN.png" width="250"/></td>
            <td><img src="./config/doc/BoWB.png" width="250"/></td>
        </tr>
        <tr>
            <td><img src="./config/doc/RangeG.png" width="250"/></td>
            <td><img src="./config/doc/RangeN.png" width="250"/></td>
            <td><img src="./config/doc/RangeB.png" width="250"/></td>
        </tr>
        <tr>
            <td><img src="./config/doc/WiLDG.png" width="250"/></td>
            <td><img src="./config/doc/WiLDN.png" width="250"/></td>
            <td><img src="./config/doc/WiLDB.png" width="250"/></td>
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
- [**Installation**](#installation)
- [**Setup**](#setup)
- [**Datasets**](#datasets)
- [**Running WiLD-Align**](#running-wild-align)
- [**TODO**](#todo)
- [**Acknowledgement**](#acknowledgement)

---

## System Architecture

WiLD-Align integrates range measurements from Wi-Fi RSSI into an inter-robot frame alignment process. Pairwise Consistency Maximization (PCM) identifies consistent constraints, and selected keyframes are refined through point cloud registration.

<p align="center">
    <img src="./config/doc/architecture.png" alt="WiLD-Align Architecture" width="800"/>
</p>

---

## Package Dependencies

- **[GTSAM](https://gtsam.org/)** — For PCM and keyframe handling  
- **[PMC]** — For Pairwise Consistency Maximization  
- **[PCL](https://pointclouds.org/)** — For point cloud processing in WiLD-Align  
- **[ROS Noetic](http://wiki.ros.org/noetic)** — Core middleware

---

## Installation

### Install message format (`WiLD-Align_msgs`)
```bash
cd ~/wild_align_msgs/src
git clone http://github.com/anonymous/WiLD-Align.git
git checkout WiLD_msgs
cd ../
catkin_make
```

### Install WiLD-Align
```bash
cd ~/wild_align_ws/src
git clone http://github.com/anonymous/WiLD-Align.git
cd ../
source ~/wild_align_msgs/devel/setup.bash
catkin_make
```

## Setup
> **Note:** The package will not run until configuration is complete.

1. **Edit file paths** to max clique logs (in `/logs`) in:
   ```
   src/pairwise_consistency_maximization/global_map/global_map.cpp
   ```
   to include your home directory.

2. **Edit `config/params.yaml`** to reflect your desired keyframe and ranging topics.

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

6. **Recompile**:
```bash
source ~/wild_align_msgs/devel/setup.bash
catkin_make
```

7. **Disable demo mode**  
Set `demoMode` in `interOptimization.cpp` to `false` for deployment.

## Datasets
Raw robot pair datasets can be found on [Google Drive](https://drive.google.com/drive/folders/1nBQmlLJm1U56DQsAWjlI_VHNSGsOqWHh?usp=sharing) — containing LiDAR, IMU, and RSSI captures from two robots.

## Running WiLD-Align
Launch the system:
```bash
source devel/setup.bash
roslaunch wild_align run.launch
```

To run in test mode with namespaces `/robot1` and `/robot2`:
```bash
roslaunch wild_align robot1.launch
roslaunch wild_align robot2.launch
```
> **Note:** You must create a communications node in ROS to facilitate inter-robot data exchange.

## TODO
- Update to ROS2 and provide CSLAM system integrations  
- Consolidate control variables into `params.yaml`  

## Acknowledgement
- WiLD-Align’s ROS framework was inspired by **[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)**  
- Uses **[Scan Context](https://github.com/irapkaist/scancontext)**  
- Uses **[Pairwise Consistency Maximization (PCM)](https://github.com/ethz-asl/pcm)**
```



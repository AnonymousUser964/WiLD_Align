#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <omp.h>

using namespace std;

typedef pcl::PointXYZI PointType;

class Params
{
public:
    int numberRobots;
    std::vector<string> rangeTopics;
    std::vector<string> keyframeTopics;
    std::vector<string> pathTopics;
    std::vector<string> cloudTopics;
    std::string keyframePublishTopic;
    std::string relativeMeasurementsTopic;

    float publishFrequency;
    float requestFrequency;
    float searchFrequency;

    ros::NodeHandle nh;

    int bowSearchWindow;
    float residualErrorThresh;
    float reistrationSearchDistance;
    float rangeNoiseBound;
    bool use2Dmapping;
    int trustRSSIThreshold;
    int thresholdTrustedRMNumber;

    bool saveMap;
    bool demoMode;
    bool reportData;
    bool verbose;

    Params()
    {
        nh.param<int>("wild_align/numberRobots", numberRobots, 2);
        if (!nh.getParam("wild_align/rangeTopics", rangeTopics)) {
            rangeTopics = {"rawr/range"};  
        }

        if (!nh.getParam("wild_align/keyframeTopics", keyframeTopics)) {
            keyframeTopics = {"wild_align/local_keyframe"};
        }

        if (!nh.getParam("wild_align/pathTopics", pathTopics)) {
            pathTopics = {"wild_align/local_path"};
        }
        if (!nh.getParam("wild_align/cloudTopics", cloudTopics)) {
            cloudTopics = {"wild_align/local_cloud"};
        }

        nh.param<float>("wild_align/publishFrequency", publishFrequency, 1);
        nh.param<float>("wild_align/requestFrequency", requestFrequency, 0.5);
        nh.param<float>("wild_align/searchFrequency", searchFrequency, 0.5);

        nh.param<std::string>("wild_align/keyframePublishTopic", keyframePublishTopic, "wild_align/pub_keyframe");
        nh.param<std::string>("wild_align/relativeMeasurementsTopic", relativeMeasurementsTopic, "wild_align/relative_measurements");
        
        nh.param<bool>("wild_align/saveMap", saveMap, false);
        nh.param<bool>("wild_align/demoMode", demoMode, false);
        nh.param<bool>("wild_align/demoMode", reportData, false);
        nh.param<bool>("wild_align/demoMode", verbose, false);
        nh.param<int>("wild_align/bowSearchWindow", bowSearchWindow, 5);

        nh.param<float>("wild_align/residualErrorThresh", residualErrorThresh, 0.3);
        nh.param<float>("wild_align/rangeNoiseBound", rangeNoiseBound, 5);
        nh.param<int>("wild_align/trustRSSIThreshold", trustRSSIThreshold, 20);
        
        

        nh.param<int>("wild_align/thresholdTrustedRMNumber", thresholdTrustedRMNumber, 5);
        nh.param<float>("wild_align/reistrationSearchDistance", reistrationSearchDistance, 5);
        nh.param<bool>("wild_align/use2Dmapping", use2Dmapping, true);

        usleep(100);
    }
};

#endif

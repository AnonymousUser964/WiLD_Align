#pragma once

#include <ros/ros.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <numeric>
#include <Eigen/Dense>
#include <queue>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <mutex>
#include "keyframeContainer.h"
#include "TrajectoryEstimator.h"

//This class defines a ranging object, which interprets range values and associates them with provided poses between 2 robots
//You should modify this to handle any range source you wich to use  
class RangeModule
{
public:
    RangeModule(const std::shared_ptr<KeyframeContainer> robot1, const std::shared_ptr<KeyframeContainer> robot2, float range, float noise, int wait, bool mapType);
    void rangeMeasurementHandler(const std_msgs::Float64MultiArrayConstPtr& msgIn);
    void findCandiates();
    double distanceToPoint(const gtsam::Pose3& pose, const Eigen::Vector3d& point);
    gtsam::Pose3 vectorToPose(const Eigen::Vector3d& vec);
    void pushQueue(std::pair<int,int> pairing);
    void popQueue();

public:
    std::shared_ptr<KeyframeContainer> robot1_ = nullptr;
    std::shared_ptr<KeyframeContainer> robot2_ = nullptr;
    const double Variance = 0.000;
    const int Threshold = 2;

    TrajectoryEstimator estimator;

    float range_;
    bool initialized = false;
    int prevRobot1Pose = -1;
    int prevRobot2Pose = -1;
    float prevRange = -1.0;
    vector<std::pair<float,std::pair<int,int>>> rangeValues;
    
    //std::queue<std::pair<int,int>> searchQ;
    std::queue<int> searchQ;
    std::queue<std::pair<int,int>> searchPairs;
    std::queue<std::pair<int,int>> prioritySearchPairs;
    std::map<std::pair<int,int>, std::pair<float, float>> rangeIndexContainer;
    std::map<std::pair<int,int>, float> rangeContainer;
    std::map<std::pair<int,int>, gtsam::SharedNoiseModel> noiseContainer;
    //std::map<float, float> errors;
    float meanNoise = 0.0;
    float epsilon = 0.6; //what percentage above the average noise is allowed to pass
    std::mutex mtx;

//GTSAM values
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values finalEstimate;
    std::vector<gtsam::Pose3> trajectoryEstimate; //This variable contains the optimized path

    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, M_PI*M_PI, 1e8, 1e8, 1e8).finished());
    gtsam::noiseModel::Diagonal::shared_ptr priorHighNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 10000.0, 10000.0, 10000.0, M_PI, M_PI, M_PI).finished());

    bool estimateCreated = false;
    int length = 0;
    float min = 5;
    float noiseBound = 10;
    bool map2d = true;
    int waitUntil = 0;

    std::map<std::pair<int,int>, bool> checkedIndexContainer; 
    int window = 2;

    std::vector<gtsam::Pose3> localTraj;
    std::vector<gtsam::Pose3> peerTraj;
    std::vector<float> rangesTest;
    std::vector<float> rangesRaw;
    std::vector<float> rangesNoise;
    std::vector<int> localInd;
    std::vector<int> peerInd;

    
    gtsam::Pose3 transformation;
    gtsam::Pose3 prevTransformation;

};

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mutex>

#include "pcm/pairwise_consistency.h"
#include "graph_utils/graph_types.h"
#include "robot_measurements/robot_local_map.h"
#include "global_map/global_map.h"

//This class defines a PCM object overseeing the loop closure outlier rejection and tracking for a single robot trajectory 
class PCMModule
{
public:
    PCMModule();
    void extendPath(int id, gtsam::Matrix covariance, gtsam::Pose3 pose, gtsam::noiseModel::Diagonal::shared_ptr odometryNoise);
    void initialize(const PCMModule& localPCM);
    void addMeasurement(int i, int j, gtsam::Pose3 relative_pose, gtsam::Matrix covarianceMatrix, gtsam::noiseModel::Diagonal::shared_ptr constraintNoise);
    void clearPath();
    void performPCM(const PCMModule& localPCM);
    float currentScore(int i, int j);
    void replace(int i, int j, gtsam::Pose3 relative_pose, gtsam::Matrix covarianceMatrix, gtsam::noiseModel::Diagonal::shared_ptr constraintNoise);

public:
    const PCMModule* localPCM_ = nullptr;


    robot_measurements::RobotLocalMap map;
    graph_utils::Transforms measurements;
    std::vector<int> acceptedMeasurements;
    std::vector<pair<int,int>> allMeasurements;
    std::vector<gtsam::noiseModel::Diagonal::shared_ptr> allMeasurementNoise;
    std::vector<gtsam::Pose3> allMeasurementPoses;
    gtsam::Pose3 prevPose;
    int prevId = -1;
    bool initialized = false;
    bool priorPlaced = false;
    std::map<std::pair<int,int>, std::pair<int, float>> check;

    //Controls PCM actions
    double pcm_threshold; //Should be between 0 and 1
    bool use_heuristics;

    std::mutex mtx;
};
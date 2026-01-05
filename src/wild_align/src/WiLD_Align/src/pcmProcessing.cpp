#include "pcmProcessing.h"




 PCMModule::PCMModule(){
    acceptedMeasurements.push_back(-1);
    pcm_threshold = 0.001; 
    use_heuristics = true;
 }

 void PCMModule::clearPath(){
   map = robot_measurements::RobotLocalMap();
   priorPlaced = false;
 }


 void PCMModule::extendPath(int id, gtsam::Matrix covariance, gtsam::Pose3 pose, gtsam::noiseModel::Diagonal::shared_ptr odometryNoise){
    if(priorPlaced){
        map.addTransform(gtsam::BetweenFactor<gtsam::Pose3>(prevId, id, prevPose.between(pose), odometryNoise), covariance);
    }else{
        graph_utils::PoseWithCovariance poseWCov;
        poseWCov.pose = pose;
        poseWCov.covariance_matrix = covariance;

        graph_utils::TrajectoryPose trajPose;
        trajPose.pose = poseWCov;
        trajPose.id = id;

        graph_utils::Trajectory traj;
        traj.start_id = id; 
        traj.end_id = id;
        traj.trajectory_poses[traj.end_id] = trajPose;

        //Initialize a PCM robot map
        graph_utils::Transforms empty_transforms;
        graph_utils::LoopClosures empty_vec;
        map = robot_measurements::RobotLocalMap(traj, empty_transforms, empty_vec);
        priorPlaced = true;
    }
    prevPose = pose;
    prevId = id;
 }

 void PCMModule::initialize(const PCMModule& localPCM){
    localPCM_ = &localPCM;
    initialized = true;
 }

 void PCMModule::addMeasurement(int i, int j, gtsam::Pose3 relative_pose, gtsam::Matrix covarianceMatrix, gtsam::noiseModel::Diagonal::shared_ptr constraintNoise){
    graph_utils::PoseWithCovariance pose;
    pose.pose = relative_pose;
    pose.covariance_matrix = covarianceMatrix;

    graph_utils::Transform measurement;
    gtsam::Key ikey, jkey;
    ikey = i;
    jkey = j;
    measurement.i = ikey;
    measurement.j = jkey;
    measurement.pose = pose;
    measurement.is_loopclosure = true;
    bool replace = check.find(std::make_pair(ikey,jkey)) != check.end();

    measurements.transforms[std::make_pair(ikey, jkey)] = measurement;
    measurements.start_id = 0;
    measurements.end_id = measurements.transforms.size();
   
    if(!replace){
      allMeasurements.push_back(std::make_pair(i, j));
      allMeasurementNoise.push_back(constraintNoise);
      allMeasurementPoses.push_back(relative_pose);
      check[std::make_pair(ikey,jkey)] = std::make_pair(allMeasurements.size()-1, covarianceMatrix(0, 0)- 0.000001);
    }else{
      int ind = check[std::make_pair(ikey,jkey)].first;
      allMeasurementNoise[ind] = constraintNoise;
      allMeasurementPoses[ind] = relative_pose;
      check[std::make_pair(ikey,jkey)] = std::make_pair(ind, covarianceMatrix(0, 0)- 0.000001);
    }
 }

 void PCMModule::replace(int i, int j, gtsam::Pose3 relative_pose, gtsam::Matrix covarianceMatrix, gtsam::noiseModel::Diagonal::shared_ptr constraintNoise){
   gtsam::Key ikey, jkey;
      ikey = i;
      jkey = j; 
   bool replace = check.find(std::make_pair(ikey,jkey)) != check.end();
    if(replace){
      graph_utils::PoseWithCovariance pose;
      pose.pose = relative_pose;
      pose.covariance_matrix = covarianceMatrix;

      graph_utils::Transform measurement;
      measurement.i = ikey;
      measurement.j = jkey;
      measurement.pose = pose;
      measurement.is_loopclosure = true;

      measurements.transforms[std::make_pair(ikey, jkey)] = measurement;
      measurements.start_id = 0;
      measurements.end_id = measurements.transforms.size();

      int ind = check[std::make_pair(ikey,jkey)].first;
      allMeasurementNoise[ind] = constraintNoise;
      allMeasurementPoses[ind] = relative_pose;
      check[std::make_pair(ikey,jkey)] = std::make_pair(ind, covarianceMatrix(0, 0)- 0.000001);
    }
 }



 float PCMModule::currentScore(int i, int j){
      gtsam::Key ikey, jkey;
      ikey = i;
      jkey = j;
      if(check.find(std::make_pair(ikey,jkey)) != check.end()){
         return check[std::make_pair(ikey,jkey)].second;
      }
      else{
         return -1.0;
      }
 }


 void PCMModule::performPCM(const PCMModule& localPCM){
    if(measurements.transforms.size() <= 0) return;
    mtx.lock();
    auto interrobot_measurements = robot_measurements::InterRobotMeasurements(measurements, '1', '2');
    auto global_map = global_map::GlobalMap(localPCM.map, map, interrobot_measurements, pcm_threshold, use_heuristics);
    auto max_clique_info = global_map.pairwiseConsistencyMaximization();
    mtx.unlock();

    acceptedMeasurements = max_clique_info.first;
 }

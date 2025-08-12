#pragma once

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <wildalign_msgs/keyframe.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <unordered_map>
#include <queue>
 
#include "Scancontext.h"
#include "pcmProcessing.h"


struct PointPose
{
    PCL_ADD_INTENSITY;              // Adds intensity field (used in some LIDAR data)    
    PCL_ADD_POINT4D;                 // Adds x, y, z, and padding (to align with 16-byte SSE requirements)

    float roll;                     //Rot X
    float pitch;                    //Rot Y
    float yaw;                      //Rot Z

    double time = -1.0;             //TimeStamp (not necessarily needed if data arrives in-order and self timing added)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensures proper memory alignment
} EIGEN_ALIGN16;                    // Enforces 16-byte alignment (important for PCL and Eigen compatibility)

typedef pcl::PointXYZI PointType;

struct RelativeMeasurement
{
    gtsam::Pose3 relativePose;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise;
    gtsam::Matrix covarianceMatrix = gtsam::Matrix::Zero(6, 6);
    double residual = -1;
};

//This class is used to define the containers that will recieve, sort, and hold keyframe data including BoW
//Inputs for this class are poses and point clouds, while outputs are GTSAM factors and indexable, ordered keyframe data
//You may need to adjust member functions and data types for this class to support your specific SLAM system
//For an instance covering the local SLAM system, this should contain every keyframe and BoW available
class KeyframeContainer
{
public:
    KeyframeContainer(int ident, const ros::Publisher& pub);
    void keyframeHandler(const wildalign_msgs::keyframeConstPtr& msgIn);
    void processKeyframe(const wildalign_msgs::keyframeConstPtr& msgIn);
    void updateTrajectory(const wildalign_msgs::keyframeConstPtr& msgIn);
    void sendData(const wildalign_msgs::keyframeConstPtr& msgIn);
    void addBoW(const wildalign_msgs::keyframeConstPtr& msgIn);
    static RelativeMeasurement calculateRelativeMeasurement(int index1, int index2, const KeyframeContainer& container1, const KeyframeContainer& container2, const std::vector<gtsam::Pose3>& trajectory = std::vector<gtsam::Pose3>());
    static void getNearPointCloud(pcl::PointCloud<PointType>::Ptr& cloud, const int& key, const KeyframeContainer &container, const int& searchNum);
    static pcl::PointCloud<PointType>::Ptr transformPointCloudPose(pcl::PointCloud<PointType>::Ptr cloudIn, PointPose* transformIn);    
    static gtsam::Pose3 pclTogtsam(PointPose pose);
    void transformTraj(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
    PointPose transformPointPose(const PointPose& input, const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
    static pcl::PointCloud<pcl::FPFHSignature33>::Ptr processCloud(const pcl::PointCloud<PointType>::Ptr& cloud);
    std::vector<bool> contains(int index) const;
    static void publishPointCloud(const KeyframeContainer& robot, ros::Publisher& publisher, const gtsam::Pose3& offset = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0),gtsam::Point3(0, 0, 0)), const std::vector<gtsam::Pose3>& trajectory = std::vector<gtsam::Pose3>(), int density = 1, bool demo = false);
    static PointPose gtsamToPCL(const gtsam::Pose3& poseIn);

    void demoHandler(const wildalign_msgs::keyframeConstPtr& msgIn);
    void setDemo(const ros::Publisher& pub);
    void demoUpdate();
    float reportSize();


public:
    int id;
    ros::Publisher publisher;
    std::vector<bool> bowOccupancy;     //track which descriptors are present
    std::vector<bool> keyOccupancy;     //track which frames are present

    std::vector<gtsam::Pose3> gtsamPoses;       //trajectory data
    pcl::PointCloud<PointPose>::Ptr keyPoses6D; //trajectory data
    std::vector<geometry_msgs::Pose> msgPoses;

    pcl::PointCloud<PointPose>::Ptr keyPoses6Estimate;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoses3DEstimate;
    std::vector<gtsam::Pose3> gtsamEstimate;   
    bool estimateCreated = false;
    bool updateEstimate = false;

    int trajectoryIteration = -1; //Tells if a trajectory has been re-optimized (+) or just extended (same number)

    bool updatedTraj = false; //Tells if a trajectory has been changed and should be republished in rvis

    SCManager bow;      //Bag of Words Container

    PCMModule pcm;


    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, M_PI*M_PI, 1e8, 1e8, 1e8).finished());
    gtsam::Matrix odomCovariance = gtsam::Matrix::Zero(6, 6);
    gtsam::Matrix priorCovariance = gtsam::Matrix::Zero(6, 6);
    ros::Time timeInfoStamp;
    double timeInfoCur;
    int highestIndex = -1;

    //std::mutex q_mtx;

    std::priority_queue<int, std::vector<int>, std::greater<int>> newKeyframes;
    std::vector<bool> occupancy;
    std::vector<pcl::PointCloud<PointType>::Ptr> pointClouds;
    std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfhVec;

    std::vector<pcl::PointCloud<PointType>::Ptr> pointCloudsDense;
    std::vector<bool> occupancyDense;
    ros::Publisher demoPub;
    bool demo = false;
    
};

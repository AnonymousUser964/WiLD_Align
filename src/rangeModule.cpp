#include "rangeModule.h"

RangeModule::RangeModule(const std::shared_ptr<KeyframeContainer> robot1, const std::shared_ptr<KeyframeContainer> robot2, float range, float noise, int wait, bool mapType)
{
    robot1_ = robot1;
    robot2_ = robot2;
    min = range;
    map2d = mapType;
    noiseBound = noise;
    waitUntil = wait;

}

double RangeModule::distanceToPoint(const gtsam::Pose3& pose, const Eigen::Vector3d& point){
    // Extract the translation component of the pose
    Eigen::Vector3d pose_position = pose.translation();

    // Compute Euclidean distance
    return (pose_position - point).norm();
}

gtsam::Pose3 RangeModule::vectorToPose(const Eigen::Vector3d& vec) {
    gtsam::Point3 translation(vec[0], vec[1], vec[2]);
    gtsam::Rot3 rotation = gtsam::Rot3::identity();  // Or use any valid rotation
    return gtsam::Pose3(rotation, translation);
}


// Assigns ranges to pose pairs and provides requested keyframe data based on trajectory data
void RangeModule::rangeMeasurementHandler(const std_msgs::Float64MultiArrayConstPtr &msgIn)
{
    //Ensure at least one of the robots is moving
    static std::vector<double> ranges;

    int robot1Pose = robot1_->highestIndex;
    int robot2Pose = robot2_->highestIndex;
    float range_val = msgIn->data[0];

    //Moving weighted average filter
    float weightSum = 0.0;
    float weightTot = 0.0;
    int window = 3;
    int cnt = 1;
    for(int i = ranges.size()-window; i < ranges.size() - 1; i++){
        if(i < 0) continue;
        weightSum += cnt*ranges[i];
        weightTot += cnt;
        cnt++;
    }
    weightSum += cnt*range_val;
    weightTot += cnt;
    ranges.push_back(weightSum/weightTot);
   

    if((robot1Pose >= 0 && robot2Pose >= 0) && (robot1Pose != prevRobot1Pose || robot2Pose != prevRobot2Pose)){
        //If the distance is "close", then automatically include it for a search
        if(ranges.back() < min){
            if(checkedIndexContainer.find({robot1Pose,robot2Pose}) == checkedIndexContainer.end()){
                mtx.lock();
                searchPairs.push({robot1Pose,robot2Pose});
                mtx.unlock();

            }
        }
        

        //Store and use new measurement
        rangeContainer[{robot1Pose,robot2Pose}] = ranges.back();
        estimator.addMeasurementConstraint(robot1_->gtsamPoses[robot1Pose].translation(),robot2_->gtsamPoses[robot2Pose].translation(), msgIn->data[0], noiseBound, map2d);

        //Create new estimate for displaying and 
        if(!estimator.first_time){
            trajectoryEstimate.clear();
            gtsam::Rot3 R(estimator.currentEstimate.first);
            gtsam::Point3 t(estimator.currentEstimate.second);
            gtsam::Pose3 T(R,t);
            for(int i = 0; i < robot2_->gtsamPoses.size(); i++){
                trajectoryEstimate.push_back(T * robot2_->gtsamPoses[i]);
            }
            estimateCreated = true;
        }


        if(estimateCreated && (rangeContainer.size() > waitUntil)) findCandiates();



        localTraj.push_back(robot1_->gtsamPoses[robot1Pose]);
        peerTraj.push_back(robot2_->gtsamPoses[robot2Pose]);
        rangesTest.push_back(ranges.back());
        rangesRaw.push_back(range_val);

        prevRobot1Pose=robot1Pose;
        prevRobot2Pose=robot2Pose;
        
        localInd.push_back(robot1Pose);
        peerInd.push_back(robot2Pose);
    }
}

void RangeModule::findCandiates(){
    std::vector<gtsam::Pose3> traj1_copy = robot1_->gtsamPoses;
    std::vector<gtsam::Pose3> traj2_copy = trajectoryEstimate;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < traj2_copy.size(); i++) {
        gtsam::Point3 t = traj2_copy[i].translation();
        cloud->points.emplace_back(t.x(), t.y(), t.z());
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    #pragma omp parallel for num_threads(4)
    for (int i = 0; i < traj1_copy.size(); i++){
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::PointXYZ point(traj1_copy[i].translation().x(),traj1_copy[i].translation().y(),traj1_copy[i].translation().z());
        if(kdtree.radiusSearch(point, min, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
             for(int j: pointIdxRadiusSearch)
             {
                 #pragma omp critical
                {
                if(checkedIndexContainer.find({i,j}) == checkedIndexContainer.end()){
                    mtx.lock();
                    searchPairs.push({i, j});
                    mtx.unlock();
                    checkedIndexContainer[{i, j}] = true;
                }
                }
             }
        }
    }

}

void RangeModule::pushQueue(std::pair<int,int> pairing){
    mtx.lock();
    searchPairs.push(pairing);
    mtx.unlock();
}

void RangeModule::popQueue(){
    mtx.lock();
    searchPairs.pop();
    mtx.unlock();
}




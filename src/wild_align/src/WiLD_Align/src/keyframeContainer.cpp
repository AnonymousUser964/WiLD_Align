#include "keyframeContainer.h"

//Register our struct with PCL for functions like kdtree and setInputCloud
POINT_CLOUD_REGISTER_POINT_STRUCT (PointPose, (float, intensity, intensity)
                                   (float, x, x) (float, y, y)(float, z, z) 
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))


void KeyframeContainer::keyframeHandler(const wildalign_msgs::keyframeConstPtr& msgIn){
    switch(msgIn->purpose){
        case 0: //request for data to be sent to origin
            sendData(msgIn);
            break;
        case 1: //recieved BoW descriptor
            addBoW(msgIn);
            break;
        case 2: //recieved keyframe data
            processKeyframe(msgIn);
            break;
        case 3: //recieved trajectory data
            updateTrajectory(msgIn);
            timeInfoStamp = msgIn->header.stamp;
            break;
        default:
            ROS_INFO("WiLD-Align::Recieved an unmarked message in KeyframeContainer %d", id);
    }
}

void KeyframeContainer::transformTraj(const Eigen::Matrix3d& R, const Eigen::Vector3d& t){
    keyPoses6Estimate->clear();
    keyPoses3DEstimate->clear();
    gtsamEstimate.clear();
    for (int i = 0; i < keyPoses6D->size(); i++){
        keyPoses6Estimate->push_back(transformPointPose(keyPoses6D->points[i], R, t));
        keyPoses3DEstimate->push_back(pcl::PointXYZ(keyPoses6Estimate->points[i].x,keyPoses6Estimate->points[i].y,keyPoses6Estimate->points[i].z));
        gtsamEstimate.push_back(pclTogtsam(keyPoses6Estimate->points[i]));
    }
    estimateCreated = true;
    updateEstimate = true;
}


PointPose KeyframeContainer::transformPointPose(const PointPose& input, const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    PointPose output = input;

    // Transform position
    Eigen::Vector3d pos(input.x, input.y, input.z);
    Eigen::Vector3d pos_transformed = R * pos + t;
    output.x = pos_transformed.x();
    output.y = pos_transformed.y();
    output.z = pos_transformed.z();

    // Build rotation matrix from roll, pitch, yaw
    Eigen::Matrix3d R_input = (
        Eigen::AngleAxisd(input.yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(input.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(input.roll,  Eigen::Vector3d::UnitX())
    ).toRotationMatrix();

    // Apply rotation
    Eigen::Matrix3d R_combined = R * R_input;

    // Convert back to roll, pitch, yaw
    Eigen::Vector3d rpy = R_combined.eulerAngles(2, 1, 0); // ZYX: yaw, pitch, roll
    output.roll  = rpy.z();
    output.pitch = rpy.y();
    output.yaw   = rpy.x();

    return output;
}


std::vector<bool> KeyframeContainer::contains(int index) const{ //Trajectory, BoW, Keyframe
    std::vector<bool> res;
    if(index < 0){
        res.push_back(false);
        res.push_back(false);
        res.push_back(false);
    }
    else{
        if(index < gtsamPoses.size()){
            res.push_back(true);
        }
        else{
            res.push_back(false);
        }
        if(index < bow.occupation.size()){
            res.push_back(bow.occupation[index]);
        }
        else{
            res.push_back(false);
        }
        if(index < keyOccupancy.size()){
            res.push_back(keyOccupancy[index]);
        }
        else{
            res.push_back(false);
        }
    }
    return res;
}

void KeyframeContainer::demoUpdate(){
    bool flag = true;
    while(flag){
        flag = false;
        wildalign_msgs::keyframe request;
        std::vector<uint32_t> keyRequest;
        for(int i = 0; i < gtsamPoses.size(); i++){
            if(i >= pointCloudsDense.size()){
                keyRequest.push_back(static_cast<uint32_t>(i));
                flag = true;
              
            }
            else if(!occupancyDense[i]){
                keyRequest.push_back(static_cast<uint32_t>(i));
                flag = true;
                
            }
        }
        if(flag){
            request.identifier = id;
            request.purpose = 0;
            request.reqKey = keyRequest;
            demoPub.publish(request);
        }
        ros::Duration(0.4).sleep();
    }
}

void KeyframeContainer::demoHandler(const wildalign_msgs::keyframeConstPtr &msgIn){
    if(msgIn->purpose == 2){
        pcl::PointCloud<PointType>::Ptr totalCloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(msgIn->cloud, *totalCloud);
        if(pointCloudsDense.size() <= msgIn->seq){
            for(int i = pointCloudsDense.size(); i < msgIn->seq - 1; i++){
                pointCloudsDense.push_back(nullptr);
                occupancyDense.push_back(false);
                
            }
            
            occupancyDense.push_back(true);
            pointCloudsDense.push_back(totalCloud);
           
        }else{
            occupancyDense[msgIn->seq]=true;
            pointCloudsDense[msgIn->seq] = totalCloud;
           
        }
    }else if(msgIn->purpose == 0) {
        for(int i = 0; i < msgIn->reqKey.size(); i++){
            if(keyOccupancy.size() <= i) continue;
            int key = msgIn->reqKey[i];
            if(keyOccupancy[key]){
                wildalign_msgs::keyframe keyframe;

                sensor_msgs::PointCloud2 totalCloud;
                pcl::toROSMsg(*pointClouds[key], totalCloud);
                keyframe.cloud = totalCloud;

                keyframe.pose.position.x = gtsamPoses[key].translation().x();
                keyframe.pose.position.y = gtsamPoses[key].translation().y();
                keyframe.pose.position.z = gtsamPoses[key].translation().z();
                keyframe.pose.orientation.x = gtsamPoses[key].rotation().roll();
                keyframe.pose.orientation.y = gtsamPoses[key].rotation().pitch();
                keyframe.pose.orientation.z = gtsamPoses[key].rotation().yaw();
                
                keyframe.header.stamp = timeInfoStamp;
                keyframe.seq = key;
                keyframe.identifier = id;
                keyframe.reqIdentifier = msgIn->identifier;
                keyframe.purpose = 2; //Add Keyframe
                demoPub.publish(keyframe);
            } 
        }

    }

}

void KeyframeContainer::setDemo(const ros::Publisher &pub){
    demoPub = pub;
    demo = true;
}

void KeyframeContainer::processKeyframe(const wildalign_msgs::keyframeConstPtr& msgIn){
    if(demo && id == 0){//Only happens for local keyframes
        wildalign_msgs::keyframe cloud;
        cloud.header = msgIn->header;
        cloud.seq = msgIn->seq;
        cloud.cloud = msgIn->cloud;
        cloud.purpose = 2;
        demoPub.publish(cloud);
    }
    
    //Collect pose data
    PointPose pose6D;
    pose6D.intensity = msgIn->seq;
    pose6D.x = msgIn->pose.position.x;
    pose6D.y = msgIn->pose.position.y;
    pose6D.z = msgIn->pose.position.z;
    pose6D.time = msgIn->header.stamp.toSec();
    pose6D.roll = msgIn->pose.orientation.x;
    pose6D.pitch = msgIn->pose.orientation.y;
    pose6D.yaw = msgIn->pose.orientation.z;

    //Collect cloud data
    pcl::PointCloud<PointType>::Ptr totalCloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(msgIn->cloud, *totalCloud);

    //Store the data
    if (highestIndex < msgIn->seq){
        for(int i = 0; i < msgIn->seq - highestIndex - 1; i++){//Increase vectors to match expected data size
            PointPose emptyPoint;
            gtsam::Pose3 pose;
            geometry_msgs::Pose msgPose;
            pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
            keyPoses6D->push_back(emptyPoint);
            pointClouds.push_back(nullptr);
            msgPoses.push_back(msgPose);
            gtsamPoses.push_back(pose);
            fpfhVec.push_back(cloud_fpfh);
            keyOccupancy.push_back(false);
        }
        keyPoses6D->push_back(pose6D);
        gtsamPoses.push_back(pclTogtsam(pose6D));
        msgPoses.push_back(msgIn->pose);
        pointClouds.push_back(totalCloud);
        fpfhVec.push_back(processCloud(totalCloud));
        keyOccupancy.push_back(true);
        highestIndex = msgIn->seq;

        timeInfoStamp = msgIn->header.stamp;
        timeInfoCur = msgIn->header.stamp.toSec();
        updatedTraj = true;

    }else{
        keyPoses6D->points[msgIn->seq] = pose6D;
        msgPoses[msgIn->seq] = msgIn->pose;
        pointClouds[msgIn->seq] = totalCloud;
        gtsamPoses[msgIn->seq] = pclTogtsam(pose6D);
        fpfhVec[msgIn->seq] = processCloud(totalCloud);
        keyOccupancy[msgIn->seq] = true;
    }
    bow.makeAndSaveScancontextAndKeys(*totalCloud,msgIn->seq);

    //Add trajectory value to PCM
    if(msgIn->seq == 0){
        pcm.extendPath(msgIn->seq,priorCovariance,gtsamPoses[msgIn->seq],priorNoise);
    }else{
        pcm.extendPath(msgIn->seq,odomCovariance,gtsamPoses[msgIn->seq],odometryNoise);
    }

    if(msgIn->poses.size()>1){ //Reoptimized trajectory
        updateTrajectory(msgIn);
    }

    newKeyframes.push(msgIn->seq);
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr KeyframeContainer::processCloud(const pcl::PointCloud<PointType>::Ptr& cloud){
    //Compute Cloud Normals
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.1);
    ne.compute(*cloud_normals);

    //Compute FPFH Features
    pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(cloud_normals);
    fpfh.setSearchMethod(tree);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.setRadiusSearch(0.2);
    fpfh.compute(*cloud_fpfh);

    return cloud_fpfh;
}

gtsam::Pose3 KeyframeContainer::pclTogtsam(PointPose pose){
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(pose.roll), double(pose.pitch), double(pose.yaw)),
                                  gtsam::Point3(double(pose.x), double(pose.y), double(pose.z)));
}

void KeyframeContainer::updateTrajectory(const wildalign_msgs::keyframeConstPtr& msgIn){
    pcm.clearPath();
    msgPoses = msgIn->poses;
    bool newTraj = trajectoryIteration < static_cast<int>(msgIn->trajectoryIter);
    for(int i = 0; i < msgIn->poses.size(); i++){
        if(i < keyOccupancy.size() && newTraj){
            keyPoses6D->points[i].intensity = i;
            keyPoses6D->points[i].x = msgIn->poses[i].position.x;
            keyPoses6D->points[i].y = msgIn->poses[i].position.y;
            keyPoses6D->points[i].z = msgIn->poses[i].position.z;
            keyPoses6D->points[i].roll = msgIn->poses[i].orientation.x;
            keyPoses6D->points[i].pitch = msgIn->poses[i].orientation.y;
            keyPoses6D->points[i].yaw = msgIn->poses[i].orientation.z;
            gtsamPoses[i] = pclTogtsam(keyPoses6D->points[i]);
        }
        else if(i >= keyOccupancy.size()){
            PointPose pose6D;
            pose6D.intensity = i;
            pose6D.x = msgIn->poses[i].position.x;
            pose6D.y = msgIn->poses[i].position.y;
            pose6D.z = msgIn->poses[i].position.z;
            pose6D.roll = msgIn->poses[i].orientation.x;
            pose6D.pitch = msgIn->poses[i].orientation.y;
            pose6D.yaw = msgIn->poses[i].orientation.z;
            keyPoses6D->push_back(pose6D);
            keyOccupancy.push_back(false);
            bowOccupancy.push_back(false);
            gtsamPoses.push_back(pclTogtsam(keyPoses6D->points[i]));

            pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
            pointClouds.push_back(nullptr);
            fpfhVec.push_back(cloud_fpfh);

            highestIndex = i;
        }
        //Add trajectory value to PCM
        if(i == 0){
            pcm.extendPath(i,priorCovariance,gtsamPoses[i],priorNoise);
        }else{
            pcm.extendPath(i,odomCovariance,gtsamPoses[i],odometryNoise);
        }
    }
    if(newTraj) trajectoryIteration = msgIn->trajectoryIter; 
    updatedTraj = true;
}


//This function should only be used for local keyframeContainers
void KeyframeContainer::sendData(const wildalign_msgs::keyframeConstPtr& msgIn){
    for(int i = 0; i < msgIn->reqBow.size(); i++){ //Publish Bag of Words descriptors
        if(bow.occupation.size() <= i) continue;
        if(bow.occupation[msgIn->reqBow[i]]){
            std::vector<double> matrices_flat;
            std::vector<uint32_t> matrix_dims;
            std::pair<std::vector<Eigen::MatrixXd>, std::vector<float>> value = bow.retrieveDescriptor(msgIn->reqBow[i]);
            for (const auto& mat : value.first) {
                uint32_t rows = mat.rows();
                uint32_t cols = mat.cols();
                matrix_dims.push_back(rows);
                matrix_dims.push_back(cols);
            
                for (int i = 0; i < rows; ++i)
                    for (int j = 0; j < cols; ++j)
                        matrices_flat.push_back(mat(i, j));
            }
            wildalign_msgs::keyframe descriptor;
            descriptor.matrices_flat = matrices_flat;
            descriptor.matrix_dims = matrix_dims; 
            descriptor.key_vec = value.second;
            descriptor.header.stamp = timeInfoStamp;
            descriptor.seq = msgIn->reqBow[i];
            descriptor.identifier = id;
            descriptor.reqIdentifier = msgIn->identifier;
            descriptor.purpose = 1; //Add Bag of Word descriptor
            publisher.publish(descriptor);
        } 
    }
    for(int i = 0; i < msgIn->reqKey.size(); i++){ //Publish Keyframes
        if(keyOccupancy.size() <= i) continue;
        int key = msgIn->reqKey[i];
        if(keyOccupancy[key]){
            wildalign_msgs::keyframe keyframe;

            sensor_msgs::PointCloud2 totalCloud;
            pcl::toROSMsg(*pointClouds[key], totalCloud);
            keyframe.cloud = totalCloud;

            keyframe.pose.position.x = gtsamPoses[key].translation().x();
            keyframe.pose.position.y = gtsamPoses[key].translation().y();
            keyframe.pose.position.z = gtsamPoses[key].translation().z();
            keyframe.pose.orientation.x = gtsamPoses[key].rotation().roll();
            keyframe.pose.orientation.y = gtsamPoses[key].rotation().pitch();
            keyframe.pose.orientation.z = gtsamPoses[key].rotation().yaw();
            
            keyframe.header.stamp = timeInfoStamp;
            keyframe.seq = key;
            keyframe.identifier = id;
            keyframe.reqIdentifier = msgIn->identifier;
            keyframe.purpose = 2; //Add Keyframe
            publisher.publish(keyframe);
        } 
    }
    if(msgIn->identifier == 0){ //Publish trajectory
        wildalign_msgs::keyframe trajectory;
        trajectory.header.stamp = timeInfoStamp;
        trajectory.identifier = id;
        trajectory.reqIdentifier = msgIn->identifier;
        trajectory.purpose = 3; //Update trajectory
        trajectory.poses = msgPoses;
        trajectory.trajectoryIter = trajectoryIteration + 1;
        publisher.publish(trajectory);
    }

}

void KeyframeContainer::addBoW(const wildalign_msgs::keyframeConstPtr& msgIn){
    //Unpack into 3 SC matrices and 1 vector
    std::vector<Eigen::MatrixXd> matrices;
    std::vector<float> polarcontext_invkey_vec;
    int offset = 0;
    if(!contains(msgIn->seq)[1]){
        for (size_t i = 0; i + 1 < msgIn->matrix_dims.size(); i += 2){
            Eigen::MatrixXd mat = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                msgIn->matrices_flat.data() + offset, msgIn->matrix_dims[i], msgIn->matrix_dims[i+1]);
            matrices.push_back(mat);
            offset += msgIn->matrix_dims[i]*msgIn->matrix_dims[i+1];
        }
        polarcontext_invkey_vec = msgIn->key_vec;
        bow.insertDescriptor(matrices, polarcontext_invkey_vec, msgIn->seq);
    }
    if(msgIn->poses.size()>1){ //Reoptimized trajectory
        updateTrajectory(msgIn);
    }
}

float KeyframeContainer::reportSize(){
    float res = 0;
    for(int i = 0; i < keyOccupancy.size(); i++){
        if(!keyOccupancy[i]) continue;
        res += pointClouds[i]->points.capacity()*sizeof(PointType);
    }
    for(int i = 0; i < bow.occupation.size(); i++){
        if(!bow.occupation[i]) continue;
        res += (bow.polarcontexts_[i].rows()*bow.polarcontexts_[i].cols()*sizeof(double));
    }
    res += msgPoses.size()*sizeof(geometry_msgs::Pose);
    return res / (1024.0 * 1024.0); //Return MB of data
}

//Find a tranformation from keyframe container1[index1] to keyframe container2[index2]
RelativeMeasurement KeyframeContainer::calculateRelativeMeasurement(int index1, int index2, const KeyframeContainer &container1, const KeyframeContainer &container2, const std::vector<gtsam::Pose3>& trajectory)
{
    RelativeMeasurement rm;
    if(index1 >= container1.keyOccupancy.size() || index2 >= container2.keyOccupancy.size())return rm;
    if(!container1.keyOccupancy[index1] || !container2.keyOccupancy[index2]) return rm;
    
    Eigen::Matrix4f transformation_RANSAC;
    if(trajectory.empty()){
        //Perform initial FPFH alignment (Long but effective RANSAC operation; consider substitute or down-sampling for performance)
        pcl::SampleConsensusInitialAlignment<PointType, PointType, pcl::FPFHSignature33> initial_alignement;
        initial_alignement.setInputSource(container2.pointClouds[index2]);
        initial_alignement.setSourceFeatures(container2.fpfhVec[index2]);
        initial_alignement.setInputTarget(container1.pointClouds[index1]);
        initial_alignement.setTargetFeatures(container1.fpfhVec[index1]);
        initial_alignement.setMaximumIterations(150);//Suggested Range is 100-300 
        initial_alignement.setMinSampleDistance(0.01);
        initial_alignement.setMaxCorrespondenceDistance(1000);
        pcl::PointCloud<PointType>::Ptr initialTransformedCloud(new pcl::PointCloud<PointType>);
        initial_alignement.align(*initialTransformedCloud);
        transformation_RANSAC = initial_alignement.getFinalTransformation();
    }
    else{
        //Use provided trajectory if available
        gtsam::Pose3 poseTo = trajectory[index2];
        gtsam::Pose3 poseFrom = container1.gtsamPoses[index1];
        gtsam::Pose3 relative = poseTo.between(poseFrom);
        transformation_RANSAC = relative.matrix().cast<float>(); 
    }

    //Set ICP alignment Variables
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setRANSACOutlierRejectionThreshold(15.0*0.5);
    icp.setRANSACIterations(10000); //Previously set to 20000
    icp.setMaxCorrespondenceDistance(15.0*2);
    icp.setMaximumIterations(300);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-7);

    //Build Larger Clouds
    pcl::PointCloud<PointType>::Ptr sourceCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetCloud(new pcl::PointCloud<PointType>());
    getNearPointCloud(sourceCloud, index2, container2, 3);
    getNearPointCloud(targetCloud, index1, container1, 25);
    //Downsample for Speed
    pcl::PointCloud<PointType>::Ptr filteredSource(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setLeafSize(0.2, 0.2, 0.2);  // Adjust leaf size as needed
    voxel_grid.setInputCloud(sourceCloud);
    voxel_grid.filter(*filteredSource);

    //Apply Initial Alignment
    pcl::PointCloud<PointType>::Ptr transformedSource(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*filteredSource, *transformedSource, transformation_RANSAC);
    //Perform ICP
    icp.setInputSource(transformedSource);
    icp.setInputTarget(targetCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
    if(icp.hasConverged()){ //If ICP did not converge, lc.residual == -1
        //Build Final Transform
        Eigen::Matrix4f transformation_ICP = icp.getFinalTransformation();
        Eigen::Matrix4f finalTransform = transformation_ICP * transformation_RANSAC;
        Eigen::Matrix4f inverseTransform = finalTransform.inverse();

        float x, y, z, roll, pitch, yaw; 
        pcl::getTranslationAndEulerAngles(Eigen::Affine3f(inverseTransform), x, y, z, roll, pitch, yaw);
        rm.relativePose = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        //Store Noise
        rm.residual = icp.getFitnessScore();
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore() + 0.000001;//Prevent underdetermined behavior in GTSAM
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        rm.constraintNoise =  gtsam::noiseModel::Diagonal::Variances(Vector6);
        for (size_t i = 0; i < 6; ++i) {
            rm.covarianceMatrix(i, i) = Vector6(i);
        }
    }
    return rm;
}

void KeyframeContainer::publishPointCloud(const KeyframeContainer& robot, ros::Publisher& publisher, const gtsam::Pose3& offset, const std::vector<gtsam::Pose3>& trajectory, int density, bool demo){
    if ((publisher.getNumSubscribers() == 0) || (robot.keyOccupancy.size() == 0)) return;
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>()); 

    int rounded = static_cast<int>(std::round(1/density));
    if(trajectory.empty()){
        for(int i = 0; i < robot.keyOccupancy.size(); i = i + rounded){
            if(demo){
                if(i >= robot.occupancyDense.size()) continue;
                if(!robot.occupancyDense[i]) continue;
                *cloudOut += *transformPointCloudPose(robot.pointCloudsDense[i],  &robot.keyPoses6D->points[i]);
            }else{
                if(!robot.contains(i)[2]) continue;
                *cloudOut += *transformPointCloudPose(robot.pointClouds[i],  &robot.keyPoses6D->points[i]);
            }

        }
    }else{
        for(int i = 0; i < robot.keyOccupancy.size(); i = i + rounded){
            PointPose pose6D = gtsamToPCL(trajectory[i]);
            if(demo){
                if(i >= robot.occupancyDense.size()-1) continue;
                if(!robot.occupancyDense[i]) continue;
                *cloudOut += *transformPointCloudPose(robot.pointCloudsDense[i],  &pose6D);
            }else{
                if(!robot.contains(i)[2]) continue;
                *cloudOut += *transformPointCloudPose(robot.pointClouds[i],  &pose6D);
            }

        }
    }

    //Apply offset
    PointPose thisPose6D = gtsamToPCL(offset);

    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*transformPointCloudPose(cloudOut, &thisPose6D), tempCloud);
    tempCloud.header.stamp = robot.timeInfoStamp;
    tempCloud.header.frame_id = "odom";
    publisher.publish(tempCloud);

}

PointPose KeyframeContainer::gtsamToPCL(const gtsam::Pose3& poseIn){
    PointPose poseOut;
    poseOut.x = poseIn.translation().x();
    poseOut.y = poseIn.translation().y();
    poseOut.z = poseIn.translation().z();
    poseOut.roll  = poseIn.rotation().roll();
    poseOut.pitch = poseIn.rotation().pitch();
    poseOut.yaw   = poseIn.rotation().yaw();
    return poseOut;
}

//Directly extracted from LIO-SAM, builds upscaled pointclouds
//Requires the key be included at least in the container
void KeyframeContainer::getNearPointCloud(pcl::PointCloud<PointType>::Ptr& cloud, const int& key, const KeyframeContainer &container, const int& searchNum){
    cloud->clear();
    int cloudSize = container.gtsamPoses.size();
    gtsam::Pose3 poseFrom = container.gtsamPoses[key];
    for (int i = -searchNum; i <= searchNum; i++){
        int keyNear = key + i;
        //Ensure all needed clouds exist
        std::vector<bool> present = container.contains(keyNear);
        if (keyNear < 0 || keyNear >= cloudSize || !present[0] || !present[2])
            continue;
        //Reposition the clouds around the key cloud at origin 
        gtsam::Pose3 poseTo = container.gtsamPoses[keyNear];
        gtsam::Pose3 poseBetween = poseFrom.between(poseTo);
        PointPose thisPose6D;
        thisPose6D.x = poseBetween.translation().x();
        thisPose6D.y = poseBetween.translation().y();
        thisPose6D.z = poseBetween.translation().z();
        thisPose6D.roll  = poseBetween.rotation().roll();
        thisPose6D.pitch = poseBetween.rotation().pitch();
        thisPose6D.yaw   = poseBetween.rotation().yaw();
        *cloud += *transformPointCloudPose(container.pointClouds[keyNear], &thisPose6D);
    }

}

//Possibly externalize number of cores
//Directly extracted from LIO-SAM, transforms a point cloud to a given pose
pcl::PointCloud<PointType>::Ptr KeyframeContainer::transformPointCloudPose(pcl::PointCloud<PointType>::Ptr cloudIn, PointPose* transformIn)
{   
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    #pragma omp parallel for num_threads(8)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}



 KeyframeContainer::KeyframeContainer(int ident, const ros::Publisher& pub){
    //Indentifier
    id = ident;
    publisher = pub;

    //Set Memory for datastructures
    keyPoses6D.reset(new pcl::PointCloud<PointPose>());
    keyPoses6Estimate.reset(new pcl::PointCloud<PointPose>());
    keyPoses3DEstimate.reset(new pcl::PointCloud<pcl::PointXYZ>());

    //Odometry noise
    gtsam::Vector6 noiseVector(6);
    noiseVector << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    for (size_t i = 0; i < 6; ++i) {
        odomCovariance(i, i) = noiseVector(i);
    }

    //Prior noise
    noiseVector << 1e-3, 1e-3, M_PI*M_PI, 1e8, 1e8, 1e8; 
    for (size_t i = 0; i < 6; ++i) {
        priorCovariance(i, i) = noiseVector(i);
    }
    
    
 }

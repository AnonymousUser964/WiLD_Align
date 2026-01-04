//Some functions of WiLD-align are inspired or adapted from TixiaoShan's LIO-SAM: https://github.com/TixiaoShan/LIO-SAM  

#include "params.h"
#include <wildalign_msgs/keyframe.h>

#include "wild_align/savePCD.h"
#include "std_msgs/String.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/RangeFactor.h> 
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <sstream>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <chrono>
#include <iostream>
#include <tuple>

#include "pcm/pairwise_consistency.h" //Adapted to work with this project; must be referenced after GTSAM
#include "graph_utils/graph_types.h"
#include "robot_measurements/robot_local_map.h"
#include "global_map/global_map.h"

#include "Scancontext.h"
#include "keyframeContainer.h"
#include "pcmProcessing.h"
#include "rangeModule.h"
#include "TrajectoryEstimator.h"
#include "RigidAlignment.h"



POINT_CLOUD_REGISTER_POINT_STRUCT (PointPose, (float, intensity, intensity)
                                   (float, x, x) (float, y, y)(float, z, z) 
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

using namespace gtsam;

class interOptimization : public Params
{
public:
    //Subscribers
    std::vector<ros::Subscriber> subDataSources;
    std::vector<ros::Subscriber> subRanges;

    //Publishers
    std::vector<ros::Publisher> pubPaths;
    std::vector<ros::Publisher> pubClouds;
    ros::Publisher pubKeyframes;
    ros::Publisher pubMeasurements;

    //Demo
    bool comprehensive = true;
    ros::Publisher pubDemoClouds, pubDemoRequests;
    ros::Subscriber subDemoClouds, subDemoRequests;
    int icp = 0;
    int icpAbove = 0;

    //Data Storage and Management
    std::vector<std::shared_ptr<KeyframeContainer>> keyframeContainers;

    //Range Storage and Calculation Objects
    std::vector<std::shared_ptr<RangeModule>> rangeModules;

    //Path Variables
    std::vector<nav_msgs::Path> paths;

    //PGO GTSAM-based optimizers
    std::vector<std::shared_ptr<RigidAlignment>> optimizers;

    //Keyframe Request Queue
    std::vector<std::queue<int>> keyframeQ;
    std::vector<std::queue<pair<int,int>>> measurementQ;
    std::vector<std::queue<pair<int,int>>> descriptorQ;
    std::vector<std::queue<pair<int,int>>> candidateQ;
    std::vector<std::mutex> keyframeMTX;
    std::vector<std::mutex> measurementMTX;
    std::vector<std::mutex> candidateMTX;
    bool requestFlag = true;

    //Services
    ros::ServiceServer saveClouds;


    ros::Subscriber subLocalKeyframes;
    ros::Subscriber subPeerKeyframes;
    ros::Subscriber subRange;
    ros::Subscriber subLocalCloud;
    ros::Subscriber subPeerCloud;

    //Publishers
    ros::Publisher pubLocalPath;
    ros::Publisher pubPeerPath;
    ros::Publisher pubLoopClosure;
    ros::Publisher pubLocalCloud;
    ros::Publisher pubPeerCloud;

    //Visualization offset
    gtsam::Pose3 offset;

    //WiLD variables
    ros::Time timeInfoStamp;

    //Given your system speed, change the queue of subscriber data to match your speed and memory needs 
    interOptimization()
    {   
        offset = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0),gtsam::Point3(25, 25, 0));
        thresholdTrustedRMNumber = -1;

        keyframeQ.resize(numberRobots-1);
        measurementQ.resize(numberRobots-1);
        descriptorQ.resize(numberRobots-1);
        candidateQ.resize(numberRobots-1);
        keyframeMTX = std::vector<std::mutex>(numberRobots-1);
        measurementMTX= std::vector<std::mutex>(numberRobots-1);
        candidateMTX= std::vector<std::mutex>(numberRobots-1);

        //Fill the subscribers and publishers
        pubKeyframes = nh.advertise<wildalign_msgs::keyframe>(keyframePublishTopic, 1);
        pubMeasurements = nh.advertise<visualization_msgs::MarkerArray>(relativeMeasurementsTopic, 1);
        saveClouds = nh.advertiseService("wild_align/savePCD", &interOptimization::saveCloudssrv, this);

        for(int i = 0; i < numberRobots; i++){
            auto container = std::make_shared<KeyframeContainer>(i, pubKeyframes);
            keyframeContainers.push_back(container);
            subDataSources.push_back(nh.subscribe<wildalign_msgs::keyframe>(keyframeTopics[i], 5, &KeyframeContainer::keyframeHandler, container.get(), ros::TransportHints().tcpNoDelay()));
            pubPaths.push_back(nh.advertise<nav_msgs::Path>(pathTopics[i], 1));
            pubClouds.push_back(nh.advertise<sensor_msgs::PointCloud2>(cloudTopics[i], 1));
            
        }

        //Define range modules for tracking and modeling trajectory alignments
        for(int i = 1; i < numberRobots; i++){
            auto rangeModule = std::make_shared<RangeModule>(keyframeContainers[0], keyframeContainers[i], reistrationSearchDistance, rangeNoiseBound, trustRSSIThreshold, use2Dmapping);
            rangeModules.push_back(rangeModule);
            subRanges.push_back(nh.subscribe<std_msgs::Float64MultiArray>(rangeTopics[i-1], 5, &RangeModule::rangeMeasurementHandler, rangeModule.get(), ros::TransportHints().tcpNoDelay()));
            auto rigid = std::make_shared<RigidAlignment>();
            optimizers.push_back(rigid);
        }


        for(int i = 0; i < numberRobots; i++){
            nav_msgs::Path path;
            path.poses.clear();
            paths.push_back(path);
        }
        //Just for demo purposes:
        pubDemoClouds = nh.advertise<wildalign_msgs::keyframe>("wild_align/pub_demo_cloud", 1); //Comment out to remove demo mode
        pubDemoRequests = nh.advertise<wildalign_msgs::keyframe>("wild_align/pub_demo_request", 1);
        keyframeContainers[0]->setDemo(pubDemoClouds);
        keyframeContainers[1]->setDemo(pubDemoRequests);
        subDemoClouds = nh.subscribe<wildalign_msgs::keyframe>("wild_align/sub_demo_cloud", 5, &KeyframeContainer::demoHandler, keyframeContainers[1].get(), ros::TransportHints().tcpNoDelay());
        subDemoRequests = nh.subscribe<wildalign_msgs::keyframe>("wild_align/sub_demo_request", 5, &KeyframeContainer::demoHandler, keyframeContainers[0].get(), ros::TransportHints().tcpNoDelay());

    }

    //This save function is adapted from LIO_SAM
    //BE CAREFUL WITH YOUR EDITS, CAN DELETE FILES RECRUSIVELY IF IMPROPERLY USED
    bool saveCloudssrv(wild_align::savePCD::Request &request, wild_align::savePCD::Response &results){
        string directory;
        std::cout << "****************************************************" << std::endl;
        std::cout << "Saving pcd files" << std::endl;
        if(request.destination.empty()) {std::cout << "No directory provided" << std::endl; return true;}
        else directory = std::getenv("HOME") + request.destination;
        std::cout << "Save destination: " << directory << std::endl;

        // create directory and remove old files;
        //WARNING, editing this can delete home directory... Ask me how I know
        int unused = system((std::string("exec rm -r ") + directory).c_str());
        unused = system((std::string("mkdir -p ") + directory).c_str());

        // save local transformations
        pcl::io::savePCDFileBinary(directory + "/localtransformations.pcd", *(keyframeContainers[0]->keyPoses6D));

        pcl::PointCloud<PointType>::Ptr CombinedMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr LocalMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr LocalMapCloudVoxel(new pcl::PointCloud<PointType>());

        for(int i = 0; i < (int)keyframeContainers[0]->keyPoses6D->size(); i++){
            if(!keyframeContainers[0]->contains(i)[2]) continue;
            *LocalMapCloud += *KeyframeContainer::transformPointCloudPose(keyframeContainers[0]->pointClouds[i], &(keyframeContainers[0]->keyPoses6D->points[i]));
        }
        *CombinedMapCloud += *LocalMapCloud;

        pcl::VoxelGrid<PointType> voxel_filter;
        if(request.resolution != 0){
            voxel_filter.setLeafSize(request.resolution,request.resolution,request.resolution);

            voxel_filter.setInputCloud(LocalMapCloud);
            voxel_filter.filter(*LocalMapCloudVoxel);
            pcl::io::savePCDFileBinary(directory + "/LocalMap.pcd", *LocalMapCloudVoxel);
        }else{
            pcl::io::savePCDFileBinary(directory + "/LocalMap.pcd", *LocalMapCloud);
            pcl::io::savePCDFileBinary(directory + "/LocalEx.pcd", *keyframeContainers[0]->pointClouds[2]);
        }
        float totalMemory = 0;
        for(int z = 1; z < numberRobots; z++){
            keyframeContainers[z]->demoUpdate();

            if(keyframeContainers[z]->estimateCreated){

                pcl::PointCloud<PointType>::Ptr PeerMapCloud(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr PeerMapCloudVoxel(new pcl::PointCloud<PointType>());

                pcl::io::savePCDFileBinary(directory + "/Aligned" + std::to_string(z) + "AlignedTransformations.pcd", *(keyframeContainers[z]->keyPoses6Estimate));
                
                for(int i = 0; i < (int)keyframeContainers[z]->keyPoses6Estimate->size(); i++){
                    if(i >= keyframeContainers[z]->occupancyDense.size()) continue;
                    if(!keyframeContainers[z]->occupancyDense[i]) continue;
                    *PeerMapCloud += *KeyframeContainer::transformPointCloudPose(keyframeContainers[z]->pointCloudsDense[i], &(keyframeContainers[z]->keyPoses6Estimate->points[i]));
                }
                *CombinedMapCloud += *PeerMapCloud;

                if(request.resolution != 0){
                    voxel_filter.setLeafSize(request.resolution,request.resolution,request.resolution);

                    voxel_filter.setInputCloud(PeerMapCloud);
                    voxel_filter.filter(*PeerMapCloudVoxel);
                    pcl::io::savePCDFileBinary(directory + "/AlignedPeer"+std::to_string(z)+"Map.pcd", *PeerMapCloudVoxel);
                }else{
                    pcl::io::savePCDFileBinary(directory + "/AlignedPeer"+std::to_string(z)+"Map.pcd", *PeerMapCloud);
                    pcl::io::savePCDFileBinary(directory + "/Peer"+std::to_string(z)+"Ex.pcd", *keyframeContainers[z]->pointCloudsDense[2]);
                }
            }
            else{
                if(keyframeContainers[z]->gtsamPoses.size() > 0){
                    pcl::io::savePCDFileBinary(directory + "/peer" + std::to_string(z) + "UnalignedTransformations.pcd", *(keyframeContainers[z]->keyPoses6D));
                }
            }
            float tempMemory = keyframeContainers[z]->reportSize();
            totalMemory += tempMemory;
            if(reportData){
                ROS_INFO("Data for robot %d is %f MB", z, tempMemory);
            }
        }
        if(reportData){
            ROS_INFO("Total data recieved is %f MB", totalMemory);
            ROS_INFO("Baseline ICP Itterations: %d", icp + icpAbove);
            ROS_INFO("Additional ICP Itterations: %d", icpAbove);
        }
        int retcomb = pcl::io::savePCDFileBinary(directory + "/FinalMap.pcd", *CombinedMapCloud);
        results.success = retcomb == 0;

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed\n" << endl;

        return true;
    }



    void requestKeyFramesSrv(std::queue<int>& requests, int z){
        std::vector<uint32_t> keyRequest;
        int numKey = requests.size();
        if(numKey == 0) return;
        for(int i = 0; i < numKey; i++){//get BoW descriptors
            if(keyframeContainers[z+1]->contains(requests.front())[2]){
                requests.pop();
            }
            else{
                keyRequest.push_back(static_cast<uint32_t>(requests.front()));
                requests.push(requests.front());
                requests.pop();
            }
        }
        wildalign_msgs::keyframe request;
        request.identifier = 1;   
        request.purpose = 0;
        request.reqKey = keyRequest;
        pubKeyframes.publish(request); 
    }


////////////////////////////////////////////////////////////////////
//Thread timing functions
////////////////////////////////////////////////////////////////////

    void communicationThread()
    {
        ros::Rate rate(publishFrequency); 
        while (ros::ok()){
            rate.sleep();  
            requestBoW();
            requestKeyFrames();
	    }
    }

    void publishThread(){
        ros::Rate rate(publishFrequency); 
        while (ros::ok()){
            rate.sleep();  
            pushTrajectory();
            publishPath();
            visualizeMeasurements();
            publishMaps();
	    }
        //Print details  
        wild_align::savePCDRequest request;
        wild_align::savePCDResponse results;
        if(!saveCloudssrv(request, results)){
            ROS_INFO("Failed in saving content");
        }
    }

    void searchThread()
    {        
        ros::Rate rate(searchFrequency);
        while (ros::ok()){
            rate.sleep();
            searchMeasurementCandidates();
	    }
    }

    void optimizationThread()
    {
        ros::Rate rate(searchFrequency); 
        while (ros::ok()){
            rate.sleep(); 
            rigidBodyAlignment();
        }
    }

    void measurementThread()
    {
        ros::Rate rate(searchFrequency); 
        while (ros::ok()){
            rate.sleep(); 
            findMeasurements();
        }
    }
    
    void publishPath(){     
        //Publish robot1 which doesn't change which source the poses come from
        if(keyframeContainers[0]->updatedTraj) {
            keyframeContainers[0]->updatedTraj = false;
            updatePath(paths[0], keyframeContainers[0]);
        }
        if (pubPaths[0].getNumSubscribers() != 0){
            paths[0].header.stamp = keyframeContainers[0]->timeInfoStamp;
            paths[0].header.frame_id = "map";
            pubPaths[0].publish(paths[0]);
        }

        for (int i = 1; i < numberRobots; i++){
            //Decide which path to display based on optimization status (range based vs optimized)
            if(keyframeContainers[i]->gtsamEstimate.size() != 0){
                if(keyframeContainers[i]->updateEstimate){
                    keyframeContainers[i]->updateEstimate = false;
                    updatePath(paths[i], keyframeContainers[i], keyframeContainers[i]->timeInfoStamp);
                }
            }
            else {
                if(keyframeContainers[i]->updatedTraj){
                    keyframeContainers[i]->updatedTraj = false;
                    updatePath(paths[i], rangeModules[i-1], keyframeContainers[i]->timeInfoStamp);
                }
            }
            if (pubPaths[i].getNumSubscribers() != 0){
                paths[i].header.stamp = keyframeContainers[0]->timeInfoStamp;
                paths[i].header.frame_id = "map";
                pubPaths[i].publish(paths[i]);
            }
        }
    }

////////////////////////////////////////////////////////////////////
//Path,Map,Measurement publishing functions
////////////////////////////////////////////////////////////////////
    void updatePath(nav_msgs::Path& path, const std::shared_ptr<KeyframeContainer>& kfCont){
        path.poses.clear();
        std::vector<geometry_msgs::Pose> vec = kfCont->msgPoses;
        for(int i = 0; i < vec.size(); i++){
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose = vec[i];
            pose_stamped.header.stamp = kfCont->timeInfoStamp;
            pose_stamped.header.frame_id = "odom";
            path.poses.push_back(pose_stamped);
        }
    }
    
    void updatePath(nav_msgs::Path& path, const std::shared_ptr<RangeModule>& rm, const ros::Time& timeInfoStamp){
        path.poses.clear();
        if(!rm->estimateCreated) return;
        std::vector<gtsam::Pose3> estimate = rm->trajectoryEstimate;
        for(int i = 0; i < estimate.size(); i++){
            updateTrajectoryMsg(estimate[i], path, timeInfoStamp, 0);
        }
    }

    void updatePath(nav_msgs::Path& path, const std::shared_ptr<KeyframeContainer>& kc, const ros::Time& timeInfoStamp){
        path.poses.clear();
        int length = kc->gtsamEstimate.size();
        if(length == 0) return;
        std::vector<gtsam::Pose3> estimate = kc->gtsamEstimate;
        for(int i = 0; i < length; i++){
            updateTrajectoryMsg(estimate[i], path, timeInfoStamp, 0);
        }
    }

    void updateTrajectoryMsg(const gtsam::Pose3& pose, nav_msgs::Path& path, const ros::Time& timeInfoStamp, float offset)
    {
        geometry_msgs::PoseStamped pose_stamped;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose.rotation().roll(), pose.rotation().pitch(), pose.rotation().yaw());
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        pose_stamped.pose.position.x = pose.translation().x() + offset;
        pose_stamped.pose.position.y = pose.translation().y() + offset;
        pose_stamped.pose.position.z = pose.translation().z();

        pose_stamped.header.stamp = timeInfoStamp;
        pose_stamped.header.frame_id = "odom";
        path.poses.push_back(pose_stamped);
    }
    void pushTrajectory(){
        if (keyframeContainers[0]->highestIndex < 0) return;
        //Tell keyframe container to publish
        wildalign_msgs::keyframe request;
        request.identifier = 0;     //Include trajectory request
        request.purpose = 0;
        //request.reqBow = std::vector<uint32_t>{keyframeContainers[0]->highestIndex}; //Can send BoW request or Keyframe request 
        keyframeContainers[0]->keyframeHandler(boost::make_shared<wildalign_msgs::keyframe const>(request));
    }

    void visualizeMeasurements()
    {
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        for(int i = 1; i < numberRobots; i++){
            if (keyframeContainers[i]->pcm.acceptedMeasurements[0] == -1) continue;
            if (keyframeContainers[i]->gtsamEstimate.size() == 0) continue;
            markerNode.header.frame_id = "odom";
            markerNode.header.stamp = timeInfoStamp;
            markerNode.action = visualization_msgs::Marker::ADD;
            markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
            markerNode.ns = "loop_nodes";
            markerNode.id = 0;
            markerNode.pose.orientation.w = 1;
            markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
            markerNode.color.r = 1; markerNode.color.g = 0; markerNode.color.b = 0.9;
            markerNode.color.a = 1;
            // loop edges
            visualization_msgs::Marker markerEdge;
            markerEdge.header.frame_id = "odom";
            markerEdge.header.stamp = timeInfoStamp;
            markerEdge.action = visualization_msgs::Marker::ADD;
            markerEdge.type = visualization_msgs::Marker::LINE_LIST;
            markerEdge.ns = "loop_edges";
            markerEdge.id = 1;
            markerEdge.pose.orientation.w = 1;
            markerEdge.scale.x = 0.1;
            markerEdge.color.r = 1; markerEdge.color.g = 0; markerEdge.color.b = 0.9;
            markerEdge.color.a = 1;
 
            std::vector<int> currentAcceptedLoopClosures = keyframeContainers[i]->pcm.acceptedMeasurements;
            std::vector<gtsam::Pose3> estimate = keyframeContainers[i]->gtsamEstimate;
            std::vector<bool> useMeasurement = optimizers[i-1]->shouldUse;

            for (int val: currentAcceptedLoopClosures)
            {
                if(!useMeasurement[val]) continue;
                auto it = keyframeContainers[i]->pcm.allMeasurements[val]; //Stable
                int key_local = it.first;
                int key_peer = it.second;
                geometry_msgs::Point p;
                p.x = estimate[key_peer].translation().x();
                p.y = estimate[key_peer].translation().y();
                p.z = estimate[key_peer].translation().z();
                markerNode.points.push_back(p);
                markerEdge.points.push_back(p);
                p.x = keyframeContainers[0]->gtsamPoses[key_local].translation().x(); //Stable
                p.y = keyframeContainers[0]->gtsamPoses[key_local].translation().y(); //Stable
                p.z = keyframeContainers[0]->gtsamPoses[key_local].translation().z(); //Stable
                markerNode.points.push_back(p);
                markerEdge.points.push_back(p);
            }     

            markerArray.markers.push_back(markerNode);
            markerArray.markers.push_back(markerEdge);     

        }
        pubMeasurements.publish(markerArray);  
    }

    void publishMaps(){
        KeyframeContainer::publishPointCloud(*keyframeContainers[0], pubClouds[0]);

        for(int i = 1; i < numberRobots; i++){
            if(keyframeContainers[i]->gtsamEstimate.size() > 0){
                KeyframeContainer::publishPointCloud(*keyframeContainers[i], pubClouds[i], gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0),gtsam::Point3(0, 0, 0)),
                                                keyframeContainers[i]->gtsamEstimate, 1, demoMode);
            }else if(rangeModules[i-1]->trajectoryEstimate.size() > 0){
                //rangeModules[i-1]->updatePeerTraj();
                KeyframeContainer::publishPointCloud(*keyframeContainers[i], pubClouds[i], gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0),gtsam::Point3(0, 0, 0)),
                                                rangeModules[i-1]->trajectoryEstimate, 1, demoMode);
            }
        }
    }

////////////////////////////////////////////////////////////////////
//Data request functions (keyframes, BoW)
////////////////////////////////////////////////////////////////////
    //Request BoW based on the proximity of trajectories aligned with ranges
    void requestBoW(){
        static std::vector<std::queue<std::pair<int,int>>> requests(numberRobots-1);
        for(int z = 0; z < numberRobots-1; z++){
            //Empty range queue into a local structure
            while(!rangeModules[z]->searchPairs.empty()){
                requests[z].push(rangeModules[z]->searchPairs.front());
                rangeModules[z]->popQueue();
            }
            //Prepare data request based on missing data 
            std::vector<uint32_t> bowRequest;
            int numReq = requests[z].size();
            if(numReq == 0) return;
            for(int i = 0; i < numReq; i++){
                //get BoW descriptors for a through search 
                std::pair<int,int> key = requests[z].front();
                bool BoWPresent = true;
                for(int j = key.second - bowSearchWindow; (j < key.second + bowSearchWindow) && (j < keyframeContainers[z+1]->gtsamPoses.size()); j++){
                    if(j < 0) continue;
                    if(!keyframeContainers[z+1]->contains(j)[1]){
                        bowRequest.push_back(static_cast<uint32_t>(j));
                        BoWPresent = false;
                    }
                }
                if(BoWPresent){
                    candidateMTX[z].lock();
                    candidateQ[z].push(key);
                    candidateMTX[z].unlock();
                }else{
                    wildalign_msgs::keyframe request;
                    request.identifier = z+1;   
                    request.purpose = 0;
                    request.reqBow = bowRequest;
                    pubKeyframes.publish(request);
                    requests[z].push(key);
                }
                requests[z].pop();
            }
        }
    }

    void requestKeyFrames(){
        for(int z = 0; z < numberRobots-1; z++){
            std::vector<uint32_t> keyRequest;
            int numKey = keyframeQ[z].size();
            if(numKey == 0) return;
            for(int i = 0; i < numKey; i++){//get BoW descriptors
                if(keyframeContainers[z+1]->contains(keyframeQ[z].front())[2]){

                    keyframeMTX[z].lock();
                    keyframeQ[z].pop();
                    keyframeMTX[z].unlock();
                }
                else{
                    keyRequest.push_back(static_cast<uint32_t>(keyframeQ[z].front()));
                    keyframeMTX[z].lock();
                    keyframeQ[z].push(keyframeQ[z].front());
                    keyframeQ[z].pop();
                    keyframeMTX[z].unlock();
                }
            }
            wildalign_msgs::keyframe request;
            request.identifier = 1;   
            request.purpose = 0;
            request.reqKey = keyRequest;
            pubKeyframes.publish(request); 
        }
    }

////////////////////////////////////////////////////////////////////
//Relative measurement functions
////////////////////////////////////////////////////////////////////
    //Uses current BoW available to discover relative measurements
    void searchMeasurementCandidates(){
        static std::vector<map<int, vector<int>>> checkedIndexContainer(numberRobots-1);
        for(int z = 0; z < numberRobots-1; z++){
            if(!candidateQ[z].empty() && requestFlag && (keyframeContainers[z+1]->pcm.acceptedMeasurements.size() < 15)){
                while(!candidateQ[z].empty() && requestFlag){
                    candidateMTX[z].lock();
                    std::pair<int,int> keys = candidateQ[z].front();
                    candidateQ[z].pop();
                    candidateMTX[z].unlock();
                    #pragma omp parallel for num_threads(8)
                    for(int i = (keys.first - bowSearchWindow); i < (keys.first + bowSearchWindow); i++){
                        if(!keyframeContainers[0]->contains(i)[1]) continue;
                        std::vector<std::pair<int, float>> peerIndex;
                        peerIndex = SCManager::detectLoopClosureID(keyframeContainers[0]->bow, keyframeContainers[z+1]->bow, 
                                                                    std::make_pair(i,keys.second), bowSearchWindow, 10);
                        if(peerIndex.size() == 0) continue;
                        
                        for(int j = 0; j < peerIndex.size(); j++){
                            if(checkedIndexContainer[z].find(i) != checkedIndexContainer[z].end()){ //Ensure no repeat measurement searches
                                auto it = std::find(checkedIndexContainer[z][i].begin(), checkedIndexContainer[z][i].end(), peerIndex[j].first);    
                                if (it != checkedIndexContainer[z][i].end()){
                                    continue;
                                }
                            }
                            #pragma omp critical
                            {
                                measurementMTX[z].lock();
                                measurementQ[z].push({i, peerIndex[j].first});
                                measurementMTX[z].unlock();
                                checkedIndexContainer[z][i].push_back(peerIndex[j].first); 
                            }                       
                        }
                        
                    }

                }
            }
    
        }
    }

    void findMeasurements(){
        //Go through the measurement queue and either calculate it or request the needed keyframes
        for(int z = 0; z < numberRobots-1; z++){
            std::vector<std::pair<int,int>> measurements;
            while (!measurementQ[z].empty()) {
                measurements.push_back(measurementQ[z].front());
                measurementMTX[z].lock();
                measurementQ[z].pop();
                measurementMTX[z].unlock();
            }
            if(measurements.size() == 0 && !keyframeContainers[z+1]->estimateCreated) continue;
            #pragma omp parallel for num_threads(8)
            for(int i = 0; i < measurements.size(); i++){
                if(!keyframeContainers[z+1]->contains(measurements[i].second)[2]){//The keyframe isn't present
                    #pragma omp critical
                    {
                    keyframeMTX[z].lock();
                    keyframeQ[z].push(measurements[i].second);
                    keyframeMTX[z].unlock();
                    
                    measurementMTX[z].lock();
                    measurementQ[z].push(measurements[i]);
                    measurementMTX[z].unlock();
                    }
                    continue;
                    
                }
                RelativeMeasurement rm;
                if((keyframeContainers[z+1]->pcm.acceptedMeasurements.size() > thresholdTrustedRMNumber) && (rangeModules[z]->trajectoryEstimate.size() > 0)){
                    rm = KeyframeContainer::calculateRelativeMeasurement(measurements[i].first, measurements[i].second, *keyframeContainers[0], *keyframeContainers[z+1], rangeModules[z]->trajectoryEstimate);
                }else{ //Use initial alignment if range trajectory not available
                    rm = KeyframeContainer::calculateRelativeMeasurement(measurements[i].first, measurements[i].second, *keyframeContainers[0], *keyframeContainers[z+1]);
                }
                icp++;
                if(verbose) ROS_INFO("Local frame %d to peer frame %d ICP residual is %f",measurements[i].first, measurements[i].second, rm.residual);
                if(rm.residual < residualErrorThresh && rm.residual != -1){
                    optimizers[z]->addRelativeMeasure((keyframeContainers[0]->gtsamPoses[measurements[i].first]*rm.relativePose.inverse()*keyframeContainers[z+1]->gtsamPoses[measurements[i].second].inverse()).matrix(), measurements[i].first,measurements[i].second);
                    keyframeContainers[z+1]->pcm.addMeasurement(measurements[i].first,measurements[i].second,rm.relativePose,rm.covarianceMatrix, rm.constraintNoise);
                }
                if(keyframeContainers[z+1]->pcm.acceptedMeasurements.size() > thresholdTrustedRMNumber){
                    requestFlag = false;
                }
            }
            //This second set of calculations fine-tunes the results of the measurements above
            if(keyframeContainers[z+1]->pcm.acceptedMeasurements.size() > 0 && keyframeContainers[z+1]->estimateCreated){
                //Recalculate current loop closures to see if there is a better alignment
                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                kdtree.setInputCloud(keyframeContainers[z+1]->keyPoses3DEstimate);
                for(int i = 0; i < keyframeContainers[0]->gtsamPoses.size(); i++){
                    std::vector<int> pointIdxRadiusSearch;
                    std::vector<float> pointRadiusSquaredDistance;
                    const gtsam::Pose3 pose = keyframeContainers[0]->gtsamPoses[i];
                    pcl::PointXYZ point(pose.translation().x(),pose.translation().y(),pose.translation().z());
                    if(kdtree.radiusSearch(point, reistrationSearchDistance, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
                        #pragma omp parallel for num_threads(8)
                        for(int j: pointIdxRadiusSearch){
                            RelativeMeasurement rm;
                            rm = KeyframeContainer::calculateRelativeMeasurement(i, j, *keyframeContainers[0], *keyframeContainers[z+1], keyframeContainers[z+1]->gtsamEstimate);
                            
                            float currentScore = keyframeContainers[z+1]->pcm.currentScore(i, j);
                            if(rm.residual >= 0) {
                                icpAbove++;
                            }else if(comprehensive){
                                measurementMTX[z].lock();
                                measurementQ[z].push({i,j});
                                measurementMTX[z].unlock();
                            }
                            #pragma omp critical
                            {
                            if(currentScore < 0){
                                if((rm.residual < residualErrorThresh && rm.residual != -1)){
                                    optimizers[z]->addRelativeMeasure((keyframeContainers[0]->gtsamPoses[i]*rm.relativePose.inverse()*keyframeContainers[z+1]->gtsamPoses[j].inverse()).matrix(), i, j);
                                    keyframeContainers[z+1]->pcm.addMeasurement(i,j,rm.relativePose,rm.covarianceMatrix, rm.constraintNoise);
                                }
                            }else if(rm.residual < currentScore){
                                optimizers[z]->replaceRelativeMeasure((keyframeContainers[0]->gtsamPoses[i]*rm.relativePose.inverse()*keyframeContainers[z+1]->gtsamPoses[j].inverse()).matrix(), i, j);
                                keyframeContainers[z+1]->pcm.replace(i,j,rm.relativePose,rm.covarianceMatrix, rm.constraintNoise);
                            }
                            }
                        }
                    }
                }
                //Prune distant measurements as they tend to contribute more error
                std::vector<int> acceptedMeasurements = keyframeContainers[z+1]->pcm.acceptedMeasurements;
                for(int iter : acceptedMeasurements){
                    //gtsam::Pose3 thisPose = keyframeContainers[z+1]->pcm.allMeasurementPoses[iter];
                    auto idxs = keyframeContainers[z+1]->pcm.allMeasurements[iter];
                    if((keyframeContainers[0]->gtsamPoses[idxs.first].translation()-keyframeContainers[z+1]->gtsamEstimate[idxs.second].translation()).norm() > 1.5*reistrationSearchDistance){
                        optimizers[z]->stopUsing(idxs.first, idxs.second);
                    }
                }
            }
            
            

        }
    }
    void printMatrix3d(const Eigen::MatrixXd& mat, const std::string& label) {
        std::stringstream ss;
        ss << "\n" << label << ":\n";
        ss << mat;
        ROS_INFO_STREAM(ss.str());
    }


    Eigen::Matrix4d Pose3ToEigenMatrix(const gtsam::Pose3& pose) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat.block<3,3>(0,0) = pose.rotation().matrix();
        mat.block<3,1>(0,3) = pose.translation().vector();
        return mat;
    }

    void rigidBodyAlignment(){
        keyframeContainers[1]->pcm.performPCM(keyframeContainers[0]->pcm);
        if(keyframeContainers[1]->pcm.acceptedMeasurements[0] != -1){
            optimizers[0]->getAlignent(keyframeContainers[1]->pcm.acceptedMeasurements);
            keyframeContainers[1]->transformTraj(optimizers[0]->R_est, optimizers[0]->t_est);
        }
    }

    
        
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wild_align");
    
    interOptimization IO;
    
    ROS_INFO("\033[---->  Inter-Robot Map Alignment.\033[0m");
    
    std::thread communicationThread(&interOptimization::communicationThread, &IO);
    std::thread publishThread(&interOptimization::publishThread, &IO);
    std::thread optimizationThread(&interOptimization::optimizationThread, &IO);
    std::thread searchThread(&interOptimization::searchThread, &IO);
    std::thread measurementThread(&interOptimization::measurementThread, &IO);

    ros::spin();
    
    communicationThread.join();
    publishThread.join();
    optimizationThread.join();
    searchThread.join();
    measurementThread.join();
    
    return 0;
}

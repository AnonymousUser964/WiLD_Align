#include <ros/ros.h>
#include <ros/duration.h>
#include <wildalign_msgs/keyframe.h>
#include <std_msgs/Float64MultiArray.h>


class Coms 
{
public:
    std::string ns1 = "robot1/";
    std::string ns2 = "robot2/";
    std::string range = "wild_align/range1";

    float robot1Range, robot2Range;
    
    ros::Subscriber subRobot1;
    ros::Subscriber subRobot2;
    ros::Subscriber subRange1;
    ros::Subscriber subRange2;
    ros::Publisher pubRobot1, pubReqRobot1;
    ros::Publisher pubRobot2, pubReqRobot2;
    
    
    ros::Subscriber subTestRobot1cloud;
    ros::Subscriber subTestRobot1request;
    ros::Subscriber subTestRobot2cloud;
    ros::Subscriber subTestRobot2request;
    
    ros::Publisher pubTestRobot1cloud;
    ros::Publisher pubTestRobot1request;
    ros::Publisher pubTestRobot2cloud;
    ros::Publisher pubTestRobot2request;

    ros::Time robot1Coms;
    ros::Time robot2Coms;
    ros::Duration coms_window; //Simulate a communication constraints

    ros::NodeHandle nh;
    
    Coms(): coms_window(1.5)
    {
    	subRobot1 = nh.subscribe<wildalign_msgs::keyframe>(ns1+"wild_align/pub_keyframe", 1, &Coms::robot1Handler, this, ros::TransportHints().tcpNoDelay());
    	subRobot2 = nh.subscribe<wildalign_msgs::keyframe>(ns2+"wild_align/pub_keyframe", 1, &Coms::robot2Handler, this, ros::TransportHints().tcpNoDelay());
     	subRange1 = nh.subscribe<std_msgs::Float64MultiArray>(ns1+range, 1, &Coms::range1Handler, this, ros::TransportHints().tcpNoDelay());
    	subRange2 = nh.subscribe<std_msgs::Float64MultiArray>(ns2+range, 1, &Coms::range2Handler, this, ros::TransportHints().tcpNoDelay());   	
        
        pubRobot1 = nh.advertise<wildalign_msgs::keyframe>(ns1+"wild_align/peer1_keyframe", 1);
    	pubRobot2 = nh.advertise<wildalign_msgs::keyframe>(ns2+"wild_align/peer1_keyframe", 1);
        pubReqRobot1 = nh.advertise<wildalign_msgs::keyframe>(ns1+"wild_align/local_keyframe", 1);
    	pubReqRobot2 = nh.advertise<wildalign_msgs::keyframe>(ns2+"wild_align/local_keyframe", 1);

        subTestRobot1cloud = nh.subscribe<wildalign_msgs::keyframe>(ns1+"wild_align/pub_demo_cloud", 1, &Coms::robot1TestCHandler, this, ros::TransportHints().tcpNoDelay());
    	subTestRobot1request = nh.subscribe<wildalign_msgs::keyframe>(ns1+"wild_align/pub_demo_request", 1, &Coms::robot1TestRHandler, this, ros::TransportHints().tcpNoDelay());
    	subTestRobot2cloud = nh.subscribe<wildalign_msgs::keyframe>(ns2+"wild_align/pub_demo_cloud", 1, &Coms::robot2TestCHandler, this, ros::TransportHints().tcpNoDelay());
    	subTestRobot2request = nh.subscribe<wildalign_msgs::keyframe>(ns2+"wild_align/pub_demo_request", 1, &Coms::robot2TestRHandler, this, ros::TransportHints().tcpNoDelay());
        
        pubTestRobot1cloud = nh.advertise<wildalign_msgs::keyframe>(ns1+"wild_align/sub_demo_cloud", 1);
    	pubTestRobot1request = nh.advertise<wildalign_msgs::keyframe>(ns1+"wild_align/sub_demo_request", 1);
    	pubTestRobot2cloud = nh.advertise<wildalign_msgs::keyframe>(ns2+"wild_align/sub_demo_cloud", 1);
    	pubTestRobot2request = nh.advertise<wildalign_msgs::keyframe>(ns2+"wild_align/sub_demo_request", 1);
    	
    }
    
    void robot1TestCHandler(const wildalign_msgs::keyframeConstPtr& msgIn){ //Pass message from robot1 to robot2
         pubTestRobot2cloud.publish(*msgIn);
    }
    void robot2TestCHandler(const wildalign_msgs::keyframeConstPtr& msgIn){ //Pass message from robot1 to robot2
         pubTestRobot1cloud.publish(*msgIn);
    }
    void robot1TestRHandler(const wildalign_msgs::keyframeConstPtr& msgIn){ //Pass message from robot1 to robot2
         pubTestRobot2request.publish(*msgIn);
    }
    void robot2TestRHandler(const wildalign_msgs::keyframeConstPtr& msgIn){ //Pass message from robot1 to robot2
         pubTestRobot1request.publish(*msgIn);
    }
    
    
    void range1Handler(const std_msgs::Float64MultiArrayConstPtr& msgIn){ //Range indicates robot1 sees robot2
        robot1Coms = ros::Time::now();
        robot1Range = msgIn->data[0];
    }

    void range2Handler(const std_msgs::Float64MultiArrayConstPtr& msgIn){ //Range indicates robot2 sees robot1
        robot2Coms = ros::Time::now();
        robot2Range = msgIn->data[0];
    }

    void robot1Handler(const wildalign_msgs::keyframeConstPtr& msgIn){ //Pass message from robot1 to robot2
    	if(((ros::Time::now() - robot2Coms) < coms_window) || robot2Range < 15){
            wildalign_msgs::keyframe msgOut = *msgIn;
            msgOut.identifier = 1;
            if(msgOut.purpose == 0){
                pubReqRobot2.publish(msgOut);
            }
            else{
                pubRobot2.publish(msgOut);
            }
        }
    }
    
    void robot2Handler(const wildalign_msgs::keyframeConstPtr& msgIn){ //Pass message from robot2 to robot1
    	if(((ros::Time::now() - robot1Coms) < coms_window) || robot1Range < 15){
            wildalign_msgs::keyframe msgOut = *msgIn;
            msgOut.identifier = 1;
            if(msgOut.purpose == 0){
                pubReqRobot1.publish(msgOut);
            }
            else{
                pubRobot1.publish(msgOut);
            }
        }
    } 
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "coms");

    Coms com;

    ROS_INFO("\033[1;32m----> Communication Simulation Node.\033[0m");

    ros::spin();


    return 0;
}


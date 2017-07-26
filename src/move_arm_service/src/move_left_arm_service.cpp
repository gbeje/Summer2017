#include "ros/ros.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/SolvePositionIKRequest.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/JointCommand.h"
#include "geometry_msgs/PoseStamped.h"
#include "stdlib.h"
#include "move_arm_service/MoveArm.h"
#include "baxter_core_msgs/EndpointState.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float64.h"

ros::ServiceClient* clientPtr; 
ros::Publisher* leftLimbPubPtr;
ros::Publisher* leftSpeedPubPtr;
//ros::Subscriber* posSubPtr;
//ros::Subscriber* rangeSubPtr;
geometry_msgs::Pose current_endpoint;
sensor_msgs::Range current_sensor;

void rangeCallback(const sensor_msgs::Range::ConstPtr& left_range){
	current_sensor.range = left_range -> range;
}

void endpointCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint){
	current_endpoint = endpoint -> pose;
}


bool move(move_arm_service::MoveArm::Request &req,
          move_arm_service::MoveArm::Response &res)
{
	ros::ServiceClient client = (ros::ServiceClient) *clientPtr;
	ros::Publisher leftLimbPub = (ros::Publisher) *leftLimbPubPtr;
	ros::Publisher leftSpeedPub = (ros::Publisher) *leftSpeedPubPtr; 
	baxter_core_msgs::SolvePositionIK IK_service;
	IK_service.request.pose_stamp.push_back(req.pose);//adds pose to the pose_stamped msg in request
	baxter_core_msgs::JointCommand left_limb;
	int i; 
	std_msgs::Float64 speed; 
	if(client.call(IK_service)) //put the request through the server to get response outcome
	{
		if(IK_service.response.isValid[0]==true)
		{
			ROS_INFO("found a solution");
			res.found=true;
			//print out solution
			for(i=0;i<7;i++){
				left_limb.names.push_back(IK_service.response.joints[0].name[i]);
				left_limb.command.push_back(IK_service.response.joints[0].position[i]);
			}			
			
			while(ros::ok){
				ros::Rate loop_rate(10);
				left_limb.mode=left_limb.POSITION_MODE;
//				ROS_INFO("moving to destination");
				speed.data= 0.20;
				leftSpeedPub.publish(speed);
				leftLimbPub.publish(left_limb);
				loop_rate.sleep();
				ros::spinOnce();
				if(fabs(current_endpoint.position.x-req.pose.pose.position.x)<0.1 &&
                                   fabs(current_endpoint.position.y-req.pose.pose.position.y)<0.1 &&
                                   fabs(current_endpoint.position.z-req.pose.pose.position.z)<0.1 ||
				   current_sensor.range < 0.5) {
//					ROS_INFO("Slowing down");
					speed.data=0.05;
					leftSpeedPub.publish(speed);
					ros::spinOnce();
					if(current_sensor.range<0.1){
						ROS_INFO("Found object");
					}
				}
				
				if(fabs(current_endpoint.position.x-req.pose.pose.position.x)<0.005 &&
				   fabs(current_endpoint.position.y-req.pose.pose.position.y)<0.005 &&
				   fabs(current_endpoint.position.z-req.pose.pose.position.z)<0.005 ||
				   current_sensor.range < 0.15) {
					ROS_INFO("%f", current_sensor.range);
					break;
				}
			}
		} else if (IK_service.response.isValid[0]==false){
			ROS_INFO("invalid pose: no solution found");
			res.found = false;
		}
	} else {
		ROS_INFO("failed to call IK service");
		return false; 
	}
	speed.data=0.2;
	leftSpeedPub.publish(speed);
	ros::spinOnce();
	return true;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"move_left_arm_server");
	ros::NodeHandle n;

        ros::ServiceClient ik_client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/left/PositionKinematicsNode/IKService",true);
	clientPtr=&ik_client;
	
	ros::Publisher leftLimbPub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 10);
	leftLimbPubPtr=&leftLimbPub;
	ros::Publisher leftSpeedPub = n.advertise<std_msgs::Float64>("/robot/limb/left/set_speed_ratio", 1000);
	leftSpeedPubPtr = &leftSpeedPub;

	ros::Subscriber positionSub = n.subscribe("/robot/limb/left/endpoint_state",1000,endpointCallback);
//	posSubPtr=&positionSub;
	ros::Subscriber rangeSub = n.subscribe("/robot/range/left_hand_range/state",1000, rangeCallback);

	ros::ServiceServer service = n.advertiseService("move_left_arm", move);
	
	ROS_INFO("Ready to move left arm");

	ros::spin();

	return 0;
}


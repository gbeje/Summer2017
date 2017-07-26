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
ros::Publisher* rightLimbPubPtr;
ros::Publisher* rightSpeedPubPtr;
//ros::Subscriber* posSubPtr;
//ros::Subscriber* rangeSubPtr;
geometry_msgs::Pose current_endpoint;
sensor_msgs::Range current_sensor;

void rangeCallback(const sensor_msgs::Range::ConstPtr& right_range){
	current_sensor.range = right_range -> range;
}

void endpointCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint){
	current_endpoint = endpoint -> pose;
}


bool move(move_arm_service::MoveArm::Request &req,
          move_arm_service::MoveArm::Response &res)
{
	ros::ServiceClient client = (ros::ServiceClient) *clientPtr;
	ros::Publisher rightLimbPub = (ros::Publisher) *rightLimbPubPtr;
	ros::Publisher rightSpeedPub = (ros::Publisher) *rightSpeedPubPtr; 
	baxter_core_msgs::SolvePositionIK IK_service;
	IK_service.request.pose_stamp.push_back(req.pose);//adds pose to the pose_stamped msg in request
	baxter_core_msgs::JointCommand right_limb;
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
				right_limb.names.push_back(IK_service.response.joints[0].name[i]);
				right_limb.command.push_back(IK_service.response.joints[0].position[i]);
			}			
			
			while(ros::ok){
				ros::Rate loop_rate(10);
				right_limb.mode=right_limb.POSITION_MODE;
//				ROS_INFO("moving to destination");
				speed.data= 0.20;
				rightSpeedPub.publish(speed);
				rightLimbPub.publish(right_limb);
				loop_rate.sleep();
				ros::spinOnce();
				if(fabs(current_endpoint.position.x-req.pose.pose.position.x)<0.1 &&
                                   fabs(current_endpoint.position.y-req.pose.pose.position.y)<0.1 &&
                                   fabs(current_endpoint.position.z-req.pose.pose.position.z)<0.1 ||
				   current_sensor.range < 0.5) {
//					ROS_INFO("Slowing down");
					speed.data=0.05;
					rightSpeedPub.publish(speed);
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
	rightSpeedPub.publish(speed);
	ros::spinOnce();
	return true;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"move_right_arm_server");
	ros::NodeHandle n;

        ros::ServiceClient ik_client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/right/PositionKinematicsNode/IKService",true);
	clientPtr=&ik_client;
	
	ros::Publisher rightLimbPub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);
	rightLimbPubPtr=&rightLimbPub;
	ros::Publisher rightSpeedPub = n.advertise<std_msgs::Float64>("/robot/limb/right/set_speed_ratio", 1000);
	rightSpeedPubPtr = &rightSpeedPub;

	ros::Subscriber positionSub = n.subscribe("/robot/limb/right/endpoint_state",1000,endpointCallback);
//	posSubPtr=&positionSub;
	ros::Subscriber rangeSub = n.subscribe("/robot/range/right_hand_range/state",1000, rangeCallback);

	ros::ServiceServer service = n.advertiseService("move_right_arm", move);
	
	ROS_INFO("Ready to move right arm");

	ros::spin();

	return 0;
}


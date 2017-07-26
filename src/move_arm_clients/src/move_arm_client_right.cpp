#include "ros/ros.h"
#include "move_arm_service/MoveArm.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "baxter_core_msgs/EndEffectorCommand.h"
#include "baxter_core_msgs/EndpointState.h"
#include <cstdlib>

//geometry_msgs::PoseStamped current_endpoint;
geometry_msgs::Pose current_endpoint; 

void Callback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint){
/*	current_endpoint.pose = endpoint -> pose;
	current_endpoint.header.stamp = endpoint -> header.stamp;
	current_endpoint.header.frame_id = endpoint -> header.frame_id;*/

	current_endpoint = endpoint -> pose;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"move_right_arm_client");
	ros::NodeHandle n;
	ros::ServiceClient clientL = n.serviceClient<move_arm_service::MoveArm>("move_left_arm");
	ros::ServiceClient clientR = n.serviceClient<move_arm_service::MoveArm>("move_right_arm");
	move_arm_service::MoveArm srv;
	ros::Publisher rightGripPub = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",1000);
	ros::Subscriber currentSub = n.subscribe("/robot/limb/right/endpoint_state",1000, Callback);
	baxter_core_msgs::EndEffectorCommand commandMsgR;
	geometry_msgs::PoseStamped start_pose;
	commandMsgR.id=65538;
	ros::spinOnce();
	sleep(2.0);
	ros::spinOnce();


// for when start_pose is Pose and not PoseStamped
	ros::spinOnce();

	start_pose.pose=current_endpoint;
	start_pose.header.stamp=ros::Time::now();
	start_pose.header.frame_id="base";
// for when start_pose is PoseStamped and not Pose
//	start_pose=current_endpoint;

	srv.request.pose.pose.position.x = 0.634588980953;
	srv.request.pose.pose.position.y = 0.0638848364753;
	srv.request.pose.pose.position.z = -0.478269359544;
	srv.request.pose.pose.orientation.x = 0.998133809214;
	srv.request.pose.pose.orientation.y = -0.000700604405327;
	srv.request.pose.pose.orientation.z = 0.0566081547089;
	srv.request.pose.pose.orientation.w = -0.0228894053748;
	srv.request.pose.header.stamp=ros::Time::now();
	srv.request.pose.header.frame_id="base";
	
	//moving to new location
	if(clientR.call(srv))
	{
		ROS_INFO("Called service to move right arm to table");
	}
	else 
	{
		ROS_ERROR("Failed to call service to move right arm to table");
		return 1;
	}


	//closing grippers	
	ROS_INFO("preparing to grip");
	commandMsgR.command=commandMsgR.CMD_PREPARE_TO_GRIP;
	rightGripPub.publish(commandMsgR);
	sleep(4.0);
	ROS_INFO("gripping");
	commandMsgR.command=commandMsgR.CMD_GRIP;
	rightGripPub.publish(commandMsgR);
	sleep(4.0);
	//moving to starting location
	srv.request.pose=start_pose; 
	if(clientR.call(srv))
        {
                ROS_INFO("Called service to move right arm to original position");
        }
        else
        {
                ROS_ERROR("Failed to call service to move right arm to original position");
                return 1;
        }
	
	//moving to different location
	srv.request.pose.pose.position.x = 0.638352210726;
        srv.request.pose.pose.position.y = -0.219849881183;
        srv.request.pose.pose.position.z = -0.179161053852;
        srv.request.pose.pose.orientation.x = 0.923726457654;
        srv.request.pose.pose.orientation.y = -0.368268785089;
        srv.request.pose.pose.orientation.z = 0.0150246393054;
        srv.request.pose.pose.orientation.w = 0.104315835679;
        srv.request.pose.header.stamp=ros::Time::now();
        srv.request.pose.header.frame_id="base";

	if(clientR.call(srv))
        {
                ROS_INFO("Called service to move right arm to new position");
        }
        else
        {
                ROS_ERROR("Failed to call service to move right arm to new position");
                return 1;
        }

	//opening grippers
	ROS_INFO("releasing");
	commandMsgR.command=commandMsgR.CMD_RELEASE;
	rightGripPub.publish(commandMsgR);
	sleep(4.0);

	ros::shutdown();

	return 0; 
}

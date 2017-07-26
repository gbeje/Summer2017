#include "ros/ros.h"
#include "move_arm_service/MoveArm.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "baxter_core_msgs/EndEffectorCommand.h"
#include "baxter_core_msgs/EndpointState.h"
#include <cstdlib>
#include "tf/transform_listener.h"

//geometry_msgs::PoseStamped current_endpoint;
/*geometry_msgs::Pose current_endpoint; */

/*void Callback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint){
//	current_endpoint.pose = endpoint -> pose;
//	current_endpoint.header.stamp = endpoint -> header.stamp;
//	current_endpoint.header.frame_id = endpoint -> header.frame_id;

	current_endpoint = endpoint -> pose;
}*/

geometry_msgs::PoseStamped tfToPoseStamped(tf::StampedTransform transform);

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"move_arms_client");
	ros::NodeHandle n;
//	ros::ServiceClient clientL = n.serviceClient<move_arm_service::MoveArm>("move_left_arm");
//	ros::ServiceClient clientR = n.serviceClient<move_arm_service::MoveArm>("move_right_arm");
	ros::ServiceClient clientL = n.serviceClient<move_arm_service::MoveArm>("move_up_left_arm");
	move_arm_service::MoveArm srv;
	ros::Publisher leftGripPub = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command",1000);
	tf::TransformListener tflistener;
//	ros::Subscriber currentSub = n.subscribe("/robot/limb/left/endpoint_state",1000, Callback);
	baxter_core_msgs::EndEffectorCommand commandMsgL;
	geometry_msgs::PoseStamped start_pose;
	tf::StampedTransform transform;
	commandMsgL.id=65538;
	ros::spinOnce();
	sleep(2.0);
	ros::spinOnce();

	try{
		tflistener.lookupTransform("base","l_gripper_r_finger_tip",ros::Time(0),transform);
	} 
	catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		return 1;
	}
	start_pose=tfToPoseStamped(transform);
/*// for when start_pose is Pose and not PoseStamped
	ros::spinOnce();

	start_pose.pose=current_endpoint;
	start_pose.header.stamp=ros::Time::now();
	start_pose.header.frame_id="base";
// for when start_pose is PoseStamped and not Pose
//	start_pose=current_endpoint;
*/
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
	if(clientL.call(srv))
	{
		ROS_INFO("Called service to move left arm to table");
	}
	else 
	{
		ROS_ERROR("Failed to call service to move left arm to table");
		return 1;
	}


	//closing grippers	
	ROS_INFO("preparing to grip");
	commandMsgL.command=commandMsgL.CMD_PREPARE_TO_GRIP;
	leftGripPub.publish(commandMsgL);
	sleep(4.0);
	ROS_INFO("gripping");
	commandMsgL.command=commandMsgL.CMD_GRIP;
	leftGripPub.publish(commandMsgL);
	sleep(4.0);
	//moving to starting location
	srv.request.pose=start_pose; 
	if(clientL.call(srv))
        {
                ROS_INFO("Called service to move left arm to original position");
        }
        else
        {
                ROS_ERROR("Failed to call service to move left arm to original position");
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

	if(clientL.call(srv))
        {
                ROS_INFO("Called service to move left arm to new position");
        }
        else
        {
                ROS_ERROR("Failed to call service to move left arm to new position");
                return 1;
        }

	//opening grippers
	ROS_INFO("releasing");
	commandMsgL.command=commandMsgL.CMD_RELEASE;
	leftGripPub.publish(commandMsgL);
	sleep(4.0);

	ros::shutdown();

	return 0; 
}

geometry_msgs::PoseStamped tfToPoseStamped(tf::StampedTransform transform){
        geometry_msgs::PoseStamped reply;
        reply.header.stamp=transform.stamp_;
        reply.header.frame_id=transform.frame_id_;
        reply.pose.position.x=transform.getOrigin().getX();
        reply.pose.position.y=transform.getOrigin().getY();
        reply.pose.position.z=transform.getOrigin().getZ();
        reply.pose.orientation.w=transform.getRotation().w();
        reply.pose.orientation.x=transform.getRotation().x();
        reply.pose.orientation.y=transform.getRotation().y();
        reply.pose.orientation.z=transform.getRotation().z();
        return reply;
}


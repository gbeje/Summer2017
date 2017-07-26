#include "ros/ros.h"
#include "move_arm_service/MoveArm.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "baxter_core_msgs/EndpointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"

geometry_msgs::Pose current_endpoint_left;
geometry_msgs::Pose current_endpoint_right;

std_msgs::Float32 red_msg;
std_msgs::Float32 green_msg;

void endpointLCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint){
	current_endpoint_left=endpoint -> pose;
}

void endpointRCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint){
        current_endpoint_right=endpoint -> pose;
}

geometry_msgs::PoseStamped PoseToStamped(geometry_msgs::Pose in_pose){
	geometry_msgs::PoseStamped my_pose;
	my_pose.pose=in_pose;
	my_pose.header.stamp=ros::Time::now();
	my_pose.header.frame_id="base";
	return my_pose;
}

void ClientCall(ros::ServiceClient myClient, move_arm_service::MoveArm mySrv){
	if(myClient.call(mySrv))
        {
                ROS_INFO("Called service %s",myClient.getService());
        } else {
                ROS_ERROR("Failed to call service %s",myClient.getService());
                return 1;
        }	
}

void SetHaloColor(std_msgs::Float32 red, std_msgs::Float32 green);{
	std_msgs::Float32 red_msg;
	std_msgs::Float32 green_msg;
	red_msg.data=red;
	green_msg.data=green;
	for(int i=0;i<10;i++){
                redPub.publish(red_msg);
                greenPub.publish(green_msg);
                ros::spinOnce();
        }

}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"intent_demo");
	ros::NodeHandle n;
	ros::ServiceClient clientL = n.serviceClient<move_arm_service::MoveArm>("move_left_arm");
	ros::ServiceClient clientR = n.serviceClient<move_arm_service::MoveArm>("move_right_arm");
	move_arm_service::MoveArm srv;

	ros::Subscriber currentPoseLSub=n.subscribe("/robot/limb/left/endpoint_state",1000,endpointLCallback);
	ros::Subscriber currentPoseRSub=n.subscribe("/robot/limb/right/endpoint_state",1000,endpointRCallback);
	geometry_msgs::PoseStamped start_pose_left;
	geometry_msgs::PoseStamped start_pose_right;
	geometry_msgs::PoseStamped example_pose_left;
	geometry_msgs::PoseStamped example_pose_right;
	sleep(2.0);
	
	ros::Publisher redPub=n.advertise<std_msgs::Float32>("/robot/sonar/head_sonar/lights/set_red_level",1);
	ros::Publisher greenPub=n.advertise<std_msgs::Float32>("/robot/sonar/head_sonar/lights/set_green_level",1);	
	std_msgs::Float32 red_msg;
	std_msgs::Float32 green_msg;

	ros::spinOnce();	
	
/*	start_pose_left.pose=current_endpoint_left;
	start_pose_left.header.stamp=ros::Time::now();
	start_pose_left.header.frame_id="base";
	start_pose_right.pose=current_endpoint_right;
	start_pose_right.header.stamp=ros::Time::now();
	start_pose_right.header.frame_id="base";
*/
	
	start_pose_left=PoseToStamped(current_endpoint_left);
	start_pose_right=PoseToStamped(current_endpoint_right);	

        example_pose_left.pose.position.x = 0.634588980953;
        example_pose_left.pose.position.y = 0.0638848364753;
        example_pose_left.pose.position.z = -0.478269359544;
        example_pose_left.pose.orientation.x = 0.998133809214;
        example_pose_left.pose.orientation.y = -0.000700604405327;
        example_pose_left.pose.orientation.z = 0.0566081547089;
        example_pose_left.pose.orientation.w = -0.0228894053748;
/*        example_pose_left.header.stamp=ros::Time::now();
        example_pose_left.header.frame_id="base";
*/
	example_pose_left=PoseToStamped(example_pose_left.pose);

	example_pose_right.pose.position.x = 0.977842069267; 
	example_pose_right.pose.position.y = 0.0390099477212;
	example_pose_right.pose.position.z = -0.0995524524229;
	example_pose_right.pose.orientation.x = 0.895705841517;
	example_pose_right.pose.orientation.y = 0.0143196714082;
	example_pose_right.pose.orientation.z = 0.416479888943;
	example_pose_right.pose.orientation.w = -0.155082218808;
/*	example_pose_right.header.stamp=ros::Time::now();
	example_pose_right.header.frame_id="base";
*/
	example_pose_right=PoseToStamped(example_pose_right.pose);

	//no show of intent
	srv.request.pose=example_pose_left;
/*	if(clientL.call(srv))
	{
		ROS_INFO("Called service to move left arm");
	} else {
		ROS_ERROR("Failed to call service to move left arm");
		return 1;
	}
*/
	ClientCall(clientL,srv);

	srv.request.pose=example_pose_right;
/*	if(clientR.call(srv))
	{
		ROS_INFO("Called service to move right arm");
	} else {
		ROS_ERROR("Failed to call service to move right arm");
		return 1; 
	}
*/
	ClientCall(clientR,srv);
	ros::spinOnce();
	
	//halo color
	ROS_INFO("brown");
/*	red_msg.data=100;
	green_msg.data=100;
	for(int i=0;i<10;i++){
		redPub.publish(red_msg);
		greenPub.publish(green_msg);
		ros::spinOnce();
	}
*/
	SetHaloColor(100,100);	
	sleep(2.0);

	srv.request.pose=start_pose_left;	
/*	if(clientL.call(srv))
        {
                ROS_INFO("Called service to move left arm");
        } else {
                ROS_ERROR("Failed to call service to move left arm");
                return 1;
        }
*/
	ClientCall(clientL,srv);
	
	ROS_INFO("green");
/*	red_msg.data=0;
        green_msg.data=100;
        for(int i=0;i<10;i++){
                redPub.publish(red_msg);
                greenPub.publish(green_msg);
		ros::spinOnce();
        }
*/
	SetHaloColor(0,100);
	sleep(2.0);

	ros::spinOnce();
	
	ROS_INFO("brown");
/*	red_msg.data=100;
        green_msg.data=100;
        for(int i=0;i<10;i++){
                redPub.publish(red_msg);
                greenPub.publish(green_msg);
		ros::spinOnce();
        }
*/
	SetHaloColor(100,100);

	srv.request.pose=start_pose_right;
/*	if(clientR.call(srv))
        {
                ROS_INFO("Called service to move left arm");
        } else {
                ROS_ERROR("Failed to call service to move left arm");
                return 1;
        }
*/
	ClientCall(clientR,srv);	
	sleep(2.0);

	ROS_INFO("green");
/*	red_msg.data=0;
        green_msg.data=100;
        for(int i=0;i<10;i++){
                redPub.publish(red_msg);
                greenPub.publish(green_msg);
        	ros::spinOnce();
	}
*/
	SetHaloColor(0,100);
	sleep(2.0);

	//sonar lights
	ros::Publisher sonarPub = n.advertise<std_msgs::UInt16>("/robot/sonar/head_sonar/lights/set_lights",1);
        ros::Rate sonar_rate(100);
        std_msgs::UInt16 light_control_msg;
		//modes
        const uint16_t DEFAULT_BEHAVIOR = 0x0000;
        const uint16_t OVERRIDE_ENABLE = 0x8000;
        	//states
        const uint16_t ALL_LIGHTS_OFF=0x8000;
        const uint16_t ALL_LIGHTS_ON=0x0fff;
        const uint16_t LED_0_ON = 0x0001;
        const uint16_t LED_1_ON = 0x0002;
        const uint16_t LED_2_ON = 0x0004;
        const uint16_t LED_3_ON = 0x0008;
        const uint16_t LED_4_ON = 0x0010;
        const uint16_t LED_5_ON = 0x0020;
        const uint16_t LED_6_ON = 0x0040;
        const uint16_t LED_7_ON = 0x0080;
        const uint16_t LED_8_ON = 0x0100;
        const uint16_t LED_9_ON = 0x0200;
        const uint16_t LED_10_ON = 0x0400;
        const uint16_t LED_11_ON = 0x0800;
        const uint16_t LED_ALL_ON = 0x8FFF;
	const uint16_t LED_ALL_OFF = 0x8000;


	light_control_msg.data=LED_ALL_OFF|LED_0_ON|LED_11_ON|LED_10_ON|LED_9_ON|LED_8_ON|LED_7_ON;
        ros::Time begin = ros::Time::now();
        while(ros::ok&&ros::Time::now().sec-begin.sec<2){
                sonarPub.publish(light_control_msg);
        }

	srv.request.pose=example_pose_left;

/*        if(clientL.call(srv))
        {
                ROS_INFO("Called service to move left arm");
        } else {
                ROS_ERROR("Failed to call service to move left arm");
                return 1;
        }
*/
	ClientCall(clientL,srv);
	
        light_control_msg.data=LED_ALL_OFF|LED_1_ON|LED_2_ON|LED_3_ON|LED_4_ON|LED_5_ON|LED_6_ON;
        begin = ros::Time::now();
        while(ros::ok&&ros::Time::now().sec-begin.sec<2){
                sonarPub.publish(light_control_msg);
        }
	srv.request.pose=example_pose_right;

/*        if(clientR.call(srv))
        {
                ROS_INFO("Called service to move right arm");
        } else {
                ROS_ERROR("Failed to call service to move right arm");
                return 1;
        }
*/
	ClientCall(clientR,srv);
	ros::spinOnce();
	
	return 0;
}

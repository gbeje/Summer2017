#include "ros/ros.h"
#include "baxter_core_msgs/DigitalOutputCommand.h"

int main (int argc, char* argv[])
{
	ros::init(argc,argv,"navigator_light_practice");
	ros::NodeHandle n; 
	ros::Publisher pub=n.advertise<baxter_core_msgs::DigitalOutputCommand>("/robot/digital_io/command",1);
	baxter_core_msgs::DigitalOutputCommand command_msg;
	sleep(2.0);
//	command_msg.name="left_itb_light_inner";
	command_msg.name="left_inner_light";
	command_msg.value=0;
/*	ros::Time begin = ros::Time::now();
	while(ros::ok&&ros::Time::now().sec-begin.sec<8){
		ROS_INFO("inner light");
		pub.publish(command_msg);	
	}
*/
	pub.publish(command_msg);
	sleep(2.0);
	command_msg.name="left_outer_light";
	pub.publish(command_msg);
	sleep(2.0);

//	command_msg.name="left_itb_light_outer"; 
	command_msg.name="left_outer_light";
	command_msg.value=1;
/*	begin=ros::Time::now();
	while(ros::ok&&ros::Time::now().sec-begin.sec<8){
		ROS_INFO("outer light");
		pub.publish(command_msg);
	}
*/
	pub.publish(command_msg);
	sleep(2.0);
	command_msg.name="left_inner_light";
	pub.publish(command_msg);
	sleep(2.0);
	command_msg.value=0;
	sleep(1.0);
	pub.publish(command_msg);	
	command_msg.name="left_outer_light";
	pub.publish(command_msg);
	sleep(1.0);
	ros::shutdown();
	return 0;
}

#include "ros/ros.h"
#include "baxter_core_msgs/HeadPanCommand.h"
//#include "baxter_core_msgs/HeadState.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "glance_head");
	ros::NodeHandle n; 
	ros::Publisher pub=n.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan",1);
	baxter_core_msgs::HeadPanCommand movemsg; 
	const double PI = 3.141592653589793238463;
	sleep(1.0);
	movemsg.speed_ratio=0.25;
	/*movemsg.target=PI/4;
	ROS_INFO("Moving screen to the left");	
	pub.publish(movemsg);
	movemsg.target=-PI/4;
	sleep(2.0);
	ROS_INFO("Moving screen to the right");
        pub.publish(movemsg);*/
	movemsg.target=0;
	pub.publish(movemsg);
	sleep(2.0);
	return 0;
}

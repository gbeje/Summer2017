#include "ros/ros.h"
#include "std_msgs/Float32.h"

/*float* currentRPtr; 
float* currentGPtr;

void rCallback(const std_msgs::Float32& level)
{
	*currentRPtr=level;
}

void gCallback(const )
{
	*currentGPtr=level;
}
*/

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "halo_control");
	ros::NodeHandle n; 
//	ros::Publisher redPub=n.advertise<std_msgs::Float32>("/robot/sonar/lights/set_red_level",1);
//	ros::Publisher greenPub=n.advertise<std_msgs::Float32>("/robot/sonar/lights/set_green_level",1);
	ros::Publisher redPub=n.advertise<std_msgs::Float32>("/robot/sonar/head_sonar/lights/set_red_level",1);
	ros::Publisher greenPub=n.advertise<std_msgs::Float32>("/robot/sonar/head_sonar/lights/set_green_level",1);
/*	ros::Subscriber redSub=n.subscribe("/robot/sonar/head_sonar/lights/red_level",1,rCallback); 
	ros::Subscriber greenSub=n.subscribe("/robot/sonar/head_sonar/lights/green_level",1,gCallback);
*/	std_msgs::Float32 red_msg; 
	std_msgs::Float32 green_msg;
/*	float currentR;
	float currentG;

	currentRPtr=&currentR;
	currentGPtr=&currentG; 
*/
	sleep(1.0);
	red_msg.data=50;
	green_msg.data=50;
	while(ros::ok){
		redPub.publish(red_msg);
		greenPub.publish(green_msg);	
	}
	sleep(2.0);
	return 0;
}

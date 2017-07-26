#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "stdint.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "sonar_light_practice");
	ros::NodeHandle n; 
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
	
	light_control_msg.data=LED_ALL_OFF|LED_1_ON|LED_2_ON|LED_3_ON|LED_4_ON|LED_5_ON|LED_6_ON;
	ros::Time begin = ros::Time::now();
	while(ros::ok&&ros::Time::now().sec-begin.sec<8){
		sonarPub.publish(light_control_msg);
	}
	light_control_msg.data=LED_ALL_OFF|LED_0_ON|LED_11_ON|LED_10_ON|LED_9_ON|LED_8_ON|LED_7_ON;
        begin = ros::Time::now();
        while(ros::ok&&ros::Time::now().sec-begin.sec<8){
                sonarPub.publish(light_control_msg);
        }
	return 0; 
}

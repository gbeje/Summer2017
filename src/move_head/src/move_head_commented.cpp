#include "ros/ros.h"
#include "baxter_core_msgs/HeadPanCommand.h"
#include "baxter_core_msgs/HeadState.h"

float* currentPtr; 

/*void Callback(const baxter_core_msgs::HeadState& hs)
{
	*currentPtr=hs.pan;

}*/


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "glance_head");
	ros::NodeHandle n; 
	ros::Publisher pub=n.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan",1);
//	ros::Subscriber sub=n.subscribe("/robot/head/head_state",1,Callback); 
	baxter_core_msgs::HeadPanCommand movemsg; 
	const double PI = 3.141592653589793238463;
	float current;
	currentPtr=&current;
	sleep(1.0);
//	movemsg.enable_pan_request-movemsg.REQUEST_PAN_ENABLE;
	movemsg.speed_ratio=0.25;
	movemsg.target=PI/4;
	ROS_INFO("Moving screen to the left");	
//	while(ros::ok){
		pub.publish(movemsg);
//		ROS_INFO("current pos: %f\ttarget pos: %f\tdifference: %f",*currentPtr,movemsg.target,*currentPtr-movemsg.target);
/*		ros::spinOnce();
		if(fabs(*currentPtr-(float)movemsg.target)<0.1){
			break;
		}
	}
*/	movemsg.target=-PI/4;
	sleep(2.0);
	ROS_INFO("Moving screen to the right");
//        while(ros::ok){
                pub.publish(movemsg);
/*		ros::spinOnce();
                if(fabs(*currentPtr-(float)movemsg.target)<0.1){
                        break;
                }
        }
*/
	sleep(2.0);
	return 0;
}

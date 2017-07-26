#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/AssemblyState.h"
#include "sensor_msgs/JointState.h"
#include "time.h"

baxter_core_msgs::JointCommand wrist_command;
float angle;
int current_time;

//void Callback(const baxter_core_msgs::AssemblyState& as)
void Callback(const sensor_msgs::JointState& js)
{
	angle=js.position[15];
	current_time=js.header.stamp.sec;
	return;
}

int main (int argc, char* argv[])
{
	ros::init(argc, argv, "twist_wrist");
	ros::NodeHandle n; 
	ros::Publisher pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command",10);
	ros::Subscriber sub = n.subscribe("/robot/joint_states",10,Callback);
	int start_time;
	wrist_command.names.push_back("right_w2");
	ROS_INFO("STARTING TWIST");
	ros::spinOnce();
	sleep(1.0);
	ros::spinOnce();
	if(angle+1.83>2.094){
		wrist_command.command.push_back(angle-0.5);
	} else {
		wrist_command.command.push_back(angle+0.5);
	}

	wrist_command.mode=wrist_command.POSITION_MODE;
	start_time=current_time;
	while(ros::ok){
		pub.publish(wrist_command);
		ros::spinOnce();
		if(fabs(wrist_command.command[0]- angle)<0.1||fabs(angle+1.5707)<0.05||fabs(angle-2.094)<0.05||current_time-start_time>7){
			break;
		}
	}
	return 0;
}



#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/AssemblyState.h"
#include "sensor_msgs/JointState.h"
#include "time.h"

baxter_core_msgs::JointCommand wrist_command;
float angle;
int current_time;
bool is_init;
//void Callback(const baxter_core_msgs::AssemblyState& as)
void Callback(const sensor_msgs::JointState& js)
{
//	angle=js.position[7];
	current_time=js.header.stamp.sec;
//	ROS_INFO("%s", js.name[7].c_str());
	if(js.name.size()>=8){
//		ROS_INFO("%s",js.name[7].c_str());
		if(js.name[7]=="left_w1"){
			is_init=true;
//			ROS_INFO("inited");
			angle=js.position[7];
		}
//		else {
//			ROS_INFO("not inited");
//		}
	} 
//	else {
//		ROS_INFO("too small");
//	}
	return;
}

int main (int argc, char* argv[])
{
	ros::init(argc, argv, "flick_wrist");
	ros::NodeHandle n; 
	ros::Publisher pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",10);
	ros::Subscriber sub = n.subscribe("/robot/joint_states",10,Callback);
	is_init=false;
	int start_time;
	float new_angle;
	wrist_command.names.push_back("left_w1");
	ROS_INFO("STARTING FLICK");
	while(is_init==false){
		ros::spinOnce();
		sleep(1.0);
		ROS_INFO("spinning");
	}
	ROS_INFO("original angle %f",angle);
	if(angle+1.83>2.094){
		new_angle=angle-0.5;
/*		wrist_command.command.push_back(angle-0.5);
		ROS_INFO("negative");
		ROS_INFO("new angle %f",new_angle);
*/
	} else {
		new_angle=angle+0.5;
/*		wrist_command.command.push_back(angle+0.5);
		ROS_INFO("positive");
		ROS_INFO("new angle %f",new_angle);
*/
	}
	wrist_command.command.push_back(new_angle);
	ROS_INFO("goal angle %f",new_angle);
	
	wrist_command.mode=wrist_command.POSITION_MODE;
	start_time=current_time;
	while(ros::ok){
		pub.publish(wrist_command);
		ros::spinOnce();
		if(fabs(wrist_command.command[0]- angle)<0.01||/*fabs(angle+1.5707)<0.05||fabs(angle-2.094)<0.05||*/current_time-start_time>7){
			if(current_time-start_time>7){
				ROS_INFO("oops");
			}
			break;
		}
	}
	ROS_INFO("Final angle %f",angle);
	return 0;
}



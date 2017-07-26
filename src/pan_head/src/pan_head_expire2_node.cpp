#include "ros/ros.h"
#include "baxter_core_msgs/HeadPanCommand.h"
#include "sensor_msgs/PointCloud.h"
//#include <std_msgs/String.h>

/* 
this node moves the head screen towards the nearest object
makes a new array of distances for the sensors on the front 
	of the robot (1-3,9-12)
index in the new array matches sensor_id of the distance
updates every time a new message is detected
keeps track of minimum distance and corresponding sensor_id
if a lower distance comes in, the values are updated
if the same sensor_id comes in, the min_distance changes
	even if the new distance is greater.
at a given moment, the minimums might not accurately reflect
	the state of the array, but more efficient and 
	unlikely to have any big effect
head moves if the same sensor or its neighbors has had the 
	min a few times (5?) consecutively 
******future possibility: if the minimums alternate between two 
	positions (neighboring) then the position should be the 
	middle instead of switching back and forth**********
*other future possibility: search array for minimum??*
^doesn't really make sense, won't improved the outcome,
distances will be outdated
*/

/*
Sonar sensor are numbered clockwise, with #12 facing forward on the robot
*/

const int LIMIT = 2;
const double PI = 3.141592653589793238463;
const float T_LIM=6;
float distances[13];
int min_sensor;
float min_distance=100;
int counter=0;
baxter_core_msgs::HeadPanCommand movemsg;
ros::Publisher pub;
ros::Subscriber sub;
int min_stamp; 

void updateArrayCallback(const sensor_msgs::PointCloud& pc){
	int i;
	int mysize=pc.channels[0].values.size();
/*	std_msgs::String dists;
	dists.data="recent dists: ";
	std_msgs::String senses;
	senses.data="recent sense: ";*/
//	std::string mystr;
	for (i=0;i<mysize;i++){
/*		dists.data.append(NumberToString(pc.channels[0].values[i]));
		senses.data.append(NumberToString(pc.channels[1].values[i]));*/
		ROS_INFO("%d:%f",(int)pc.channels[0].values[i],pc.channels[1].values[i]);
	}
/*	ROS_INFO(dists.data.c_str());
	ROS_INFO(senses.data.c_str());*/
	ROS_INFO("------------");
	for (i=0;i<mysize;i++){
		distances[(int)pc.channels[0].values[i]]=pc.channels[1].values[i];
		//checking if this sensor is in front of the robot
		if (pc.channels[0].values[i]<=3||pc.channels[0].values[i]>=9){
			//if a closer distance is detected
			if (pc.channels[1].values[i]<min_distance){
				min_stamp=pc.header.stamp.sec;
				ROS_INFO("new min distance %f from sensor %d old was %f from sensor %d",pc.channels[1].values[i],(int)pc.channels[0].values[i],min_distance,min_sensor);
				if (pc.channels[0].values[i]<=min_sensor+1&&pc.channels[0].values[i]>=min_sensor-1){
				//if the sensor for this distance is the same as the previous closest or one of its neighbors
					counter++;			
				} else {
					counter=0;
				}
				min_distance=pc.channels[1].values[i];
				min_sensor=pc.channels[0].values[i];
//				ROS_INFO("same min distance %f",min_distance);
			}	
			//if the current sensor_id is the same as that of the minimum
			//(in case that the object moves farther away from that sensor)
			if(pc.channels[0].values[i]==min_sensor){
				min_distance=pc.channels[1].values[i];
				min_stamp=pc.header.stamp.sec;
			}
//			ROS_INFO("dealing with message from sensor %d count is %d",(int)pc.channels[0].values[i],counter);
		} else {
                //for the case that sensor is on back of robot
//                      ROS_INFO("Irrelevant sensor %d", (int)pc.channels[0].values[i]);
                        return;
                }

//		if (counter>=LIMIT){
			//move head
			if(min_sensor<6){
				movemsg.target=PI/6*(-1)*min_sensor;
			} else { 
				movemsg.target=PI/6*(-1)*(min_sensor-12);
			}
			movemsg.speed_ratio=0.25;
			movemsg.enable_pan_request=movemsg.REQUEST_PAN_ENABLE;
//			ROS_INFO("target is: %f because sensor is: %d",movemsg.target,(int)min_sensor);	
//			pub.publish(movemsg);
//		} 
		pub.publish(movemsg);
//EXPIRATION
/*		if (pc.header.stamp.sec-min_stamp>=T_LIM){
			//resetting (expired)
			min_distance=100;
			counter=0;	
			ROS_INFO("expired");
		} */
	} 
}

int main (int argc, char* argv[]){
	ros::init(argc,argv,"follow_user");
	ros::NodeHandle n;
	pub=n.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan",1);
	sub=n.subscribe("/robot/sonar/head_sonar/state",1,updateArrayCallback);

	ros::spin();

	return 0;
}

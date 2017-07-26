#include "ros/ros.h"
#include "baxter_core_msgs/HeadPanCommand.h"
#include "sensor_msgs/PointCloud.h"

/*
this version has expiration then start over
NOPE - finding minimum not starting over
small sliding window
array is organized so that the sensor#s correspond to the
array as follows:
i=[0,1,2, 3, 4, 5,6]
#=[3,2,1,12,11,10,9]
if #<=3, i=3-#
if #> 3, i=15-#
*/

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
^saving timestamps to check if outdated
*/

/*
Sonar sensor are numbered clockwise, with #12 facing forward on the robot
         6   
     5       7 
   4           8
R 3             9 L 
   2           10
     1       11
         12
       front
*/
// minimum number of consecutive times that a sensor has to have
//the minimum before head moves to that position
const int LIMIT = 2;
// expiration limit (seconds)
const int T_LIM = 3;
// expiration limit (messages received) 
const int M_LIM =3; 
const double PI = 3.141592653589793238463;
float distances[7]=100;
int stamps[7];
//# (id) of sensor with current minimum
int min_sensor;
// current minimum distance
float min_distance=100;
int min_stamp;
// how many times the current minimum sensor has had the minimum since it
//last became the minimum
int counter=0;
// how many times a message has been received since the minimum was chosen
int received=0;
baxter_core_msgs::HeadPanCommand movemsg;
ros::Publisher pub;
ros::Subscriber sub;
// time stamp of most recent minimum reading
int min_stamp; 
// time stamp of most recent message
int last_stamp;

void findMin(void){
	int i;
	Bool found = 0;
	for (i=0;i<distances.size();i++){
		if(distances[i]<min_distance){
			if(last_stamp-stamps[i]<T_LIM){
				min_distance=distances[i];
				if(i>2){
					min_sensor=15-i;
				} else {
					min_sensor=3-i;
				}
				min_stamp=stamps[i];
				found=1;
			}
		}
	}
	if (found==1){
		ROS_INFO("Found new min: %f from sensor: %d",min_distance,min_sensor);
	} else {
		//Resetting
		min_distance=100;
		counter=0;
		ROS_INFO("No recent measurements were found. Minimum is reset");
	}
}


void updateArrayCallback(const sensor_msgs::PointCloud& pc){
	//index to look through received message
	int i;
	//index to store distances so that they are in the same order
	//as the physical sensors
	int j;
	//number of values in most recently received message
	int mysize=pc.channels[0].values.size();
	last_stamp=pc.header.stamp.sec;
	received++;
	for (i=0;i<mysize;i++){
// not recording all sensor data anymore, only front sensors
// 		distances[(int)pc.channels[0].values[i]]=pc.channels[1].values[i];
		//checking if this sensor is in front of the robot
		if (pc.channels[0].values[i]<=3||pc.channels[0].values[i]>=9){
			//index is no longer sensor number, rather so that the data is in
			//order, left to right
			// ie array is organized so that the sensor#s correspond to the
			//array as follows:
			//i=[0,1,2, 3, 4, 5,6]
			//#=[3,2,1,12,11,10,9]

			if(pc.channels[0].values[i]<=3){
				j=(int)pc.channels[0].values[i]*(-1)+3;
			} else {
				j=((int)pc.channels[0].values[i])*(-1)+15;
			}
			distances[j]=pc.channels[1].values[i];
			stamps[j]=pc.header.stamp.sec;
			//if a closer distance is detected
			if (pc.channels[1].values[i]<min_distance){
				min_stamp=pc.header.stamp.sec;
				ROS_INFO("new min distance %f from sensor %d",pc.channels[1].values[i],(int)pc.channels[0].values[i]);
				if (pc.channels[0].values[i]<=min_sensor+1&&pc.channels[0].values[i]>=min_sensor-1){
				//if the sensor for this distance is the same as the previous closest or one of its neighbors
					counter++;			
				} else {
					counter=0;
				}
				received=0;
				min_distance=pc.channels[1].values[i];
				min_sensor=pc.channels[0].values[i];
//				ROS_INFO("same min distance %f",min_distance);
			}	
			//if the current sensor_id is the same as that of the minimum
			//(in case the object moves farther away from that sensor)
			if(pc.channels[0].values[i]==min_sensor){
				min_distance=pc.channels[1].values[i];
				find_min();
				received=0;
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
		if (stamps[i]-min_stamp>=T_LIM){
			//resetting (expired)
			min_distance=100;
			counter=0;	
			ROS_INFO("expired");
//			findMin();
		}
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

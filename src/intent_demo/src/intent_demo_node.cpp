#include "ros/ros.h"
#include "move_arm_service/MoveArm.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "baxter_core_msgs/EndpointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "baxter_core_msgs/DigitalOutputCommand.h"
#include "baxter_core_msgs/HeadPanCommand.h"
#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/JointCommand.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "baxter_core_msgs/EndEffectorCommand.h"
//#include "tf/transform_datatypes.h"
//#include "tf/Transform.h"
#include "tf/transform_listener.h"



geometry_msgs::Pose current_endpoint_left;
geometry_msgs::Pose current_endpoint_right;
ros::ServiceClient* clientL;
ros::ServiceClient* clientR;
ros::Publisher sonarPub;
ros::Publisher redPub;
ros::Publisher greenPub; 
ros::Publisher navPub;
ros::Publisher panPub;
ros::Publisher leftJointPub;
ros::Publisher rightJointPub;
image_transport::Publisher* itPubPtr;
ros::Publisher leftGripPub;
ros::Publisher rightGripPub;
float leftAngle; 
float rightAngle; 
int current_joint_time;
bool left_wrist_init;
bool right_wrist_init;
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


void endpointLCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint);
void endpointRCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint);
geometry_msgs::PoseStamped PoseToStamped(geometry_msgs::Pose in_pose);
void jointCallback(const sensor_msgs::JointState& js);
bool ClientCall(ros::ServiceClient* myClient, move_arm_service::MoveArm mySrv);
void SetHaloColor(float red, float green);
void LEDLeft(void);
void LEDRight(void);
void navLeft(void);
void navRight(void);
void panLeft(void);
void panRight(void);
void point(geometry_msgs::PoseStamped start_pose,ros::ServiceClient* myClient);
void TwitchLeft(void);
void TwitchRight(void);
void visualizeTopLeft(void);
void visualizeTopRight(void);
void visualizeBottomLeft(void);
void visualizeBottomRight(void);
void gazeTopLeft(void);
void gazeTopRight(void);
void gazeBottomLeft(void);
void gazeBottomRight(void);
void prepLeft(void);
void prepRight(void);
void gripLeft(void);
void gripRight(void);
void releaseLeft(void);
void releaseRight(void);
geometry_msgs::PoseStamped tfToPoseStamped(tf::StampedTransform transform);

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"intent_demo");
	ros::NodeHandle n;
	ros::ServiceClient myClientL = n.serviceClient<move_arm_service::MoveArm>("move_left_arm");
	ros::ServiceClient myClientR = n.serviceClient<move_arm_service::MoveArm>("move_right_arm");
	clientL=&myClientL;
	clientR=&myClientR;
	tf::TransformListener tflistener;
	move_arm_service::MoveArm srv;

//	ros::Subscriber currentPoseLSub=n.subscribe("/robot/limb/left/endpoint_state",1000,endpointLCallback);
//	ros::Subscriber currentPoseRSub=n.subscribe("/robot/limb/right/endpoint_state",1000,endpointRCallback);
	ros::Subscriber jointSub=n.subscribe("/robot/joint_states",10,jointCallback);
	geometry_msgs::PoseStamped start_pose_left;
	geometry_msgs::PoseStamped start_pose_right;
	geometry_msgs::PoseStamped example_pose_left;
	geometry_msgs::PoseStamped example_pose_right;
	sleep(2.0);
	
	redPub=n.advertise<std_msgs::Float32>("/robot/sonar/head_sonar/lights/set_red_level",1);
	greenPub=n.advertise<std_msgs::Float32>("/robot/sonar/head_sonar/lights/set_green_level",1);
	navPub=n.advertise<baxter_core_msgs::DigitalOutputCommand>("/robot/digital_io/command",1);
	panPub=n.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan",1);
	leftJointPub=n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",10);
	rightJointPub=n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command",10);
	image_transport::ImageTransport it(n);
	image_transport::Publisher itPub = it.advertise("/robot/xdisplay",1);
	itPubPtr=&itPub;
	leftGripPub=n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command",1);
	rightGripPub=n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",1);
	ros::spinOnce();	
	
//	start_pose_left=PoseToStamped(current_endpoint_left);
//	start_pose_right=PoseToStamped(current_endpoint_right);	
	tf::StampedTransform transform;
	try{
		tflistener.lookupTransform("base","l_gripper_r_finger_tip",ros::Time(0),transform);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
//		continue;
		return 1;
	}
	start_pose_left=tfToPoseStamped(transform);
	try{
		tflistener.lookupTransform("base","r_gripper_r_finger_tip",ros::Time(0),transform);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
//		continue;
		return 1;
	}
	start_pose_right=tfToPoseStamped(transform);
//tf::StampedTransform
        example_pose_left.pose.position.x = 0.634588980953;
        example_pose_left.pose.position.y = 0.0638848364753;
        example_pose_left.pose.position.z = -0.478269359544;
        example_pose_left.pose.orientation.x = 0.998133809214;
        example_pose_left.pose.orientation.y = -0.000700604405327;
        example_pose_left.pose.orientation.z = 0.0566081547089;
        example_pose_left.pose.orientation.w = -0.0228894053748;
	example_pose_left=PoseToStamped(example_pose_left.pose);

	example_pose_right.pose.position.x = 0.977842069267; 
	example_pose_right.pose.position.y = 0.0390099477212;
	example_pose_right.pose.position.z = -0.0995524524229;
	example_pose_right.pose.orientation.x = 0.895705841517;
	example_pose_right.pose.orientation.y = 0.0143196714082;
	example_pose_right.pose.orientation.z = 0.416479888943;
	example_pose_right.pose.orientation.w = -0.155082218808;
	example_pose_right=PoseToStamped(example_pose_right.pose);

	//no show of intent
	srv.request.pose=example_pose_left;
	ClientCall(clientL,srv);

	srv.request.pose=example_pose_right;
	ClientCall(clientR,srv);
	ros::spinOnce();

	//halo color
	ROS_INFO("brown");
	SetHaloColor(100,100);	
	sleep(2.0);

	srv.request.pose=start_pose_left;	
	ClientCall(clientL,srv);
	
	ROS_INFO("green");
	SetHaloColor(0,100);
	sleep(2.0);

	ros::spinOnce();
	
	ROS_INFO("brown");
	SetHaloColor(100,100);

	srv.request.pose=start_pose_right;
	ClientCall(clientR,srv);	
	sleep(2.0);

	ROS_INFO("green");
	SetHaloColor(0,100);
	sleep(2.0);

	//sonar lights
	sonarPub = n.advertise<std_msgs::UInt16>("/robot/sonar/head_sonar/lights/set_lights",1);
        ros::Rate sonar_rate(100);
	LEDLeft();
	srv.request.pose=example_pose_left;
	ClientCall(clientL,srv);
	LEDRight();
	srv.request.pose=example_pose_right;
	ClientCall(clientR,srv);
	ros::spinOnce();
	
	//lights on arm
	navLeft();	
	srv.request.pose=start_pose_left;
	ClientCall(clientL,srv);
	ros::spinOnce();
	navRight();
	srv.request.pose=start_pose_right;
	ClientCall(clientR,srv);
	ros::spinOnce();

	//look (head moves in direction of motion)
	panLeft();
	ClientCall(clientL,srv);
	ros::spinOnce();
	panRight();
	srv.request.pose=example_pose_right;
	ClientCall(clientR,srv);
	ros::spinOnce();

	//point (not pointing toward target object, just move straight up and down)
	point(PoseToStamped(current_endpoint_left),clientL);
	srv.request.pose=start_pose_left;
	ClientCall(clientL,srv);
	point(PoseToStamped(current_endpoint_right),clientR);
	srv.request.pose=start_pose_right;
	ClientCall(clientR,srv);	

	//twitch (no ik)
	sleep(2.0);
	TwitchLeft();
	sleep(1.0);
	srv.request.pose=example_pose_left;
	ClientCall(clientL,srv);
	TwitchRight();
	sleep(1.0);
	srv.request.pose=example_pose_right;
	ClientCall(clientR,srv);
	
	//visualization
	sleep(2.0);
	visualizeTopLeft();
	sleep(1.0);
	srv.request.pose=start_pose_left;
	ClientCall(clientL,srv);
	visualizeTopRight();
	sleep(1.0);
	srv.request.pose=start_pose_right;
	ClientCall(clientR,srv);
	visualizeBottomLeft();
	sleep(1.0);
	srv.request.pose=example_pose_left;
	ClientCall(clientL,srv);
	visualizeBottomRight();
	sleep(1.0);
	srv.request.pose=example_pose_right;
	ClientCall(clientR,srv);

	//gaze
	sleep(2.0);
        gazeTopLeft();
        sleep(1.0);
        srv.request.pose=example_pose_left;
        ClientCall(clientL,srv);
        gazeTopRight();
        sleep(1.0);
        srv.request.pose=example_pose_right;
        ClientCall(clientR,srv);
        gazeBottomLeft();
        sleep(1.0);
        srv.request.pose=start_pose_left;
        ClientCall(clientL,srv);
        gazeBottomRight();
        sleep(1.0);
        srv.request.pose=start_pose_right;
        ClientCall(clientR,srv);
	return 0;
}


void endpointLCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint){
        current_endpoint_left=endpoint -> pose;
}

void endpointRCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint){
        current_endpoint_right=endpoint -> pose;
}

void jointCallback(const sensor_msgs::JointState& js){
        if(js.name.size()>=8&&js.name[7]=="left_w1"){
		leftAngle=js.position[7];
		left_wrist_init=true;
	}
	if(js.name.size()>=15&&js.name[14]=="right_w1"){
		rightAngle=js.position[14];
		right_wrist_init=true;
	}
        current_joint_time=js.header.stamp.sec;
        return;
}

geometry_msgs::PoseStamped PoseToStamped(geometry_msgs::Pose in_pose){
        geometry_msgs::PoseStamped my_pose;
        my_pose.pose=in_pose;
	my_pose.header.stamp=ros::Time::now();
        my_pose.header.frame_id="base";
        return my_pose;
}

bool ClientCall(ros::ServiceClient* myClient, move_arm_service::MoveArm mySrv){
        ros::ServiceClient Client=(ros::ServiceClient) *myClient;
//      move_arm_service::MoveArm Srv = (move_arm_service::MoveArm) *mySrv;
        if(Client.call(mySrv))
        {
                ROS_INFO("Called service %s",Client.getService().c_str());
                return 0;
        } else {
                ROS_ERROR("Failed to call service %s",Client.getService().c_str());
                return 1;
        }
}

void SetHaloColor(float red, float green){
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

void LEDLeft(void){
        std_msgs::UInt16 light_control_msg;
        light_control_msg.data=LED_ALL_OFF|LED_0_ON|LED_11_ON|LED_10_ON|LED_9_ON|LED_8_ON|LED_7_ON;
        ros::Time begin = ros::Time::now();
        while(ros::ok&&ros::Time::now().sec-begin.sec<2){
                sonarPub.publish(light_control_msg);
        }

}

void LEDRight(void){
        std_msgs::UInt16 light_control_msg;
        light_control_msg.data=LED_ALL_OFF|LED_1_ON|LED_2_ON|LED_3_ON|LED_4_ON|LED_5_ON|LED_6_ON;
        ros::Time begin = ros::Time::now();
        while(ros::ok&&ros::Time::now().sec-begin.sec<2){
                sonarPub.publish(light_control_msg);
        }

}

void navLeft(void){
        sleep(2.0);
        baxter_core_msgs::DigitalOutputCommand inner_command_msg;
        baxter_core_msgs::DigitalOutputCommand outer_command_msg;
        inner_command_msg.name="left_inner_light";
        inner_command_msg.value=1;
        outer_command_msg.name="left_outer_light";
        outer_command_msg.value=1;
        navPub.publish(inner_command_msg);
        navPub.publish(outer_command_msg);
        sleep(2.0);
        inner_command_msg.value=0;
        outer_command_msg.value=0;
	/////////????????baxter gripper command
        navPub.publish(inner_command_msg);
        navPub.publish(outer_command_msg);
        return;
}

void navRight(void){
        sleep(2.0);
        baxter_core_msgs::DigitalOutputCommand inner_command_msg;
        baxter_core_msgs::DigitalOutputCommand outer_command_msg;
        inner_command_msg.name="right_inner_light";
        inner_command_msg.value=1;
        outer_command_msg.name="right_outer_light";
        outer_command_msg.value=1;
        navPub.publish(inner_command_msg);
        navPub.publish(outer_command_msg);
        sleep(2.0);
        inner_command_msg.value=0;
        outer_command_msg.value=0;
        navPub.publish(inner_command_msg);
        navPub.publish(outer_command_msg);
        return;
}

void panLeft(void){
        baxter_core_msgs::HeadPanCommand msg;
        const double PI = 3.141592653589793238463;
        sleep(1.0);
        msg.speed_ratio=0.25;
        msg.target=PI/4;
        panPub.publish(msg);
        sleep(1.0);
        msg.target=0;
        panPub.publish(msg);
        return;
}

void panRight(void){
        baxter_core_msgs::HeadPanCommand msg;
        const double PI = 3.141592653589793238463;
        sleep(1.0);
        msg.speed_ratio=0.25;
        msg.target=-PI/4;
        panPub.publish(msg);
        msg.target=0;
        panPub.publish(msg);
        return;
}

void point(geometry_msgs::PoseStamped start_pose,ros::ServiceClient* myClient){
	// new_pose is where the hand will go to	
	geometry_msgs::PoseStamped new_pose;
	move_arm_service::MoveArm srv;
	//cast from the 
//	ros::ServiceClient client = (ros::ServiceClient) clientPtr;
		

	new_pose = start_pose;
	new_pose.pose.position.z += .1;

	srv.request.pose = new_pose;
/*	if(client.call(srv)){
		ROS_INFO("pointing");
		
	}else{
		ROS_INFO("failed to point");
    	}
*/
	ClientCall(myClient,srv);
	srv.request.pose = start_pose;
/*        if(client.call(srv)){
                ROS_INFO("pointing");

        }else{
                ROS_INFO("failed to point");
        }
*/
	ClientCall(myClient,srv);
	return; 
}

void TwitchLeft(void){
	baxter_core_msgs::JointCommand wrist_command;
	int start_time;
	float new_angle;
        wrist_command.names.push_back("left_w1");
        sleep(1.0);
	while(left_wrist_init!=true){
	        ros::spinOnce();
		sleep(1.0);
	}
        if(leftAngle+1.83>2.094){
                new_angle=leftAngle-0.5;
        } else {
		new_angle=leftAngle+0.5;
        }
	wrist_command.command.push_back(new_angle);
        wrist_command.mode=wrist_command.POSITION_MODE;
        start_time=current_joint_time;
        while(ros::ok){
                leftJointPub.publish(wrist_command);
                ros::spinOnce();
                if(fabs(wrist_command.command[0]-leftAngle)<0.1||current_joint_time-start_time>7){
                        if(current_joint_time-start_time>7){
                                ROS_INFO("oops - took too long");
                        }
			if(fabs(wrist_command.command[0]-leftAngle)<0.1){
				ROS_INFO("reached joint destination");
			}
                        break;
                }
        }
}

void TwitchRight(void){
	baxter_core_msgs::JointCommand wrist_command;
	int start_time;
	float new_angle;
	wrist_command.names.push_back("right_w1");
	sleep(1.0);
	while(right_wrist_init!=true){
		ros::spinOnce();
		sleep(1.0);
	}
	if(rightAngle+1.83>2.094){
		new_angle=rightAngle-0.5;
	} else {
		new_angle=rightAngle+0.5;
	}
	wrist_command.command.push_back(new_angle);
	wrist_command.mode=wrist_command.POSITION_MODE;
	start_time=current_joint_time;
	while(ros::ok){
		rightJointPub.publish(wrist_command);
		ros::spinOnce();
		if(fabs(wrist_command.command[0]-rightAngle)<0.1||current_joint_time-start_time>7){
			if(current_joint_time-start_time>7){
				ROS_INFO("oops - took too long");
			}
			if(fabs(wrist_command.command[0]-rightAngle)<0.1){
				ROS_INFO("reached joint destination");
			}
			break;
		}
	}
}


void visualizeTopLeft(void){
	image_transport::Publisher pub = (image_transport::Publisher) *itPubPtr;
	std::string tmp="/home/csrobot/Desktop/boxes_top_right.jpg";
	cv::Mat image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
	pub.publish(msg);
	ros::spinOnce(); 
	ROS_INFO("top left");	
	tmp="/home/csrobot/Desktop/blank.jpg";
	image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
	msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
	sleep(2.0);
	pub.publish(msg);
	ros::spinOnce();
	ROS_INFO("blank");
	ros::spinOnce();
	sleep(2.0);
}

void visualizeTopRight(void){
        image_transport::Publisher pub = (image_transport::Publisher) *itPubPtr;
        std::string tmp="/home/csrobot/Desktop/boxes_top_left.jpg";
        cv::Mat image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("top right");
        tmp="/home/csrobot/Desktop/blank.jpg";
        image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
        sleep(2.0);
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("blank");
        sleep(2.0);
	ros::spinOnce();
}

void visualizeBottomLeft(void){
        image_transport::Publisher pub = (image_transport::Publisher) *itPubPtr;
        std::string tmp="/home/csrobot/Desktop/boxes_bottom_right.jpg";
        cv::Mat image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("bottom left");
        tmp="/home/csrobot/Desktop/blank.jpg";
        image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
        sleep(2.0);
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("blank");
        sleep(2.0);
	ros::spinOnce();
}

void visualizeBottomRight(void){
        image_transport::Publisher pub = (image_transport::Publisher) *itPubPtr;
        std::string tmp="/home/csrobot/Desktop/boxes_bottom_left.jpg";
        cv::Mat image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("bottom right");
        tmp="/home/csrobot/Desktop/blank.jpg";
        image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
        sleep(2.0);
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("blank");
        sleep(2.0);
	ros::spinOnce();
}

void gazeTopLeft(void){
	image_transport::Publisher pub = (image_transport::Publisher) *itPubPtr;
        std::string tmp="/home/csrobot/Desktop/NeutralSEBlue.jpg";
        cv::Mat image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("top left");
        tmp="/home/csrobot/Desktop/blank.jpg";
        image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
        sleep(2.0);
        pub.publish(msg);
        ROS_INFO("blank");
        ros::spinOnce();
	sleep(2.0);
	ros::spinOnce();
}

void gazeTopRight(void){
	image_transport::Publisher pub = (image_transport::Publisher) *itPubPtr;
        std::string tmp="/home/csrobot/Desktop/NeutralSWRed.jpg";
        cv::Mat image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("top right");
        tmp="/home/csrobot/Desktop/blank.jpg";
        image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
        sleep(2.0);
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("blank");
        sleep(2.0);
	ros::spinOnce();
}

void gazeBottomLeft(void){
	image_transport::Publisher pub = (image_transport::Publisher) *itPubPtr;
        std::string tmp="/home/csrobot/Desktop/NeutralSERed.jpg";
        cv::Mat image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("bottom left");
        tmp="/home/csrobot/Desktop/blank.jpg";
        image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
        sleep(2.0);
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("blank");
        sleep(2.0);
	ros::spinOnce();
}

void gazeBottomRight(void){
	image_transport::Publisher pub = (image_transport::Publisher) *itPubPtr;
        std::string tmp="/home/csrobot/Desktop/NeutralSWBlue.jpg";
        cv::Mat image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        pub.publish(msg);
	ros::spinOnce();
        ROS_INFO("bottom right");
        tmp="/home/csrobot/Desktop/blank.jpg";
        image=cv::imread(tmp,CV_LOAD_IMAGE_COLOR);
        msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
        sleep(2.0);
        pub.publish(msg);
        ROS_INFO("blank");
        ros::spinOnce();
	sleep(2.0);
	ros::spinOnce();

}

void prepLeft(void){
	baxter_core_msgs::EndEffectorCommand msg;
	msg.id=65538;
	msg.command=msg.CMD_PREPARE_TO_GRIP;
	leftGripPub.publish(msg);
	sleep(1.0);
}

void prepRight(void){
	baxter_core_msgs::EndEffectorCommand msg;
        msg.id=65538;
        msg.command=msg.CMD_PREPARE_TO_GRIP;
        rightGripPub.publish(msg);
        sleep(1.0);
}

void gripLeft(void){
	baxter_core_msgs::EndEffectorCommand msg;
        msg.id=65538;
        msg.command=msg.CMD_GRIP;
        leftGripPub.publish(msg);
        sleep(1.0);
}

void gripRight(void){
	baxter_core_msgs::EndEffectorCommand msg;
        msg.id=65538;
        msg.command=msg.CMD_GRIP;
        leftGripPub.publish(msg);
        sleep(1.0);
}

void releaseLeft(void){
        baxter_core_msgs::EndEffectorCommand msg;
        msg.id=65538;
        msg.command=msg.CMD_RELEASE;
        leftGripPub.publish(msg);
        sleep(1.0);
}

void releaseRight(void){
        baxter_core_msgs::EndEffectorCommand msg;
        msg.id=65538;
        msg.command=msg.CMD_RELEASE;
        leftGripPub.publish(msg);
        sleep(1.0);
}

geometry_msgs::PoseStamped tfToPoseStamped(tf::StampedTransform transform){
	geometry_msgs::PoseStamped reply;
	reply.header.stamp=transform.stamp_;
	reply.header.frame_id=transform.frame_id_;
	reply.pose.position.x=transform.getOrigin().getX();
	reply.pose.position.y=transform.getOrigin().getY();
	reply.pose.position.z=transform.getOrigin().getZ();
	reply.pose.orientation.w=transform.getRotation().w();
	reply.pose.orientation.x=transform.getRotation().x();
	reply.pose.orientation.y=transform.getRotation().y();
	reply.pose.orientation.z=transform.getRotation().z();
	return reply;
}

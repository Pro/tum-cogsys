#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_tum_msgs/ActorVec.h"
#include "ros_tum_msgs/Actor.h"
#include "geometry_msgs/Pose.h"
#include "robot_control/MoveToOS.h"
#include "re_speech_recognition/LoadGrammar.h"
#include "re_speech_recognition/RecognizedSpeech.h"
#include "re_speech_recognition/TurnOn.h"
#include "re_speech_recognition/TurnOff.h"

#include "robot_control/MoveToOS.h"
#include "gripper_control/CloseGripper.h"
#include "gripper_control/OpenGripper.h"

#include <sstream>
#include <cstdlib>

struct Ros_Localizer {
	ros::ServiceClient closeGripper;
	ros::ServiceClient openGripper;
	ros::ServiceClient moveToOs;

	ros::NodeHandle n;
	ros::ServiceClient speechTurnOn;
	ros::ServiceClient speechTurnOff;
	ros::ServiceClient speechLoadGrammar;

	char property[32];
	bool taken;

};

struct Ros_Localizer *loc;
using namespace std;

void  opengripper()
{
	gripper_control::OpenGripper grpOpen;
	std::cout << "Opening Gripper" << std::endl;
	if (loc->openGripper.call(grpOpen))
	{
		if(grpOpen.response.success)
	    		std::cout << "OpenGripper successfull " <<std::endl;
		else
	    		std::cout << "OpenGripper failed" << std::endl;
	}
	else
	{
		ROS_ERROR("Failed to call service /gripper_control/open_gripper");
		throw;
	}
}

void closegripper()
{
	gripper_control::CloseGripper grpClose;
	std::cout << "Closing Gripper" << std::endl;
	if (loc->closeGripper.call(grpClose))
	{
		if(grpClose.response.success)
	    		std::cout << "CloseGripper successfull " <<std::endl;
		else
	    		std::cout << "CloseGripper failed" << std::endl;
	}
	else
	{
		ROS_ERROR("Failed to call service /gripper_control/close_gripper");
		throw;
	}
}

bool moverobot(float x, float y, float z)
{
	robot_control::MoveToOS gripperbot_srv;

	gripperbot_srv.request.x=x;
	gripperbot_srv.request.y=y;
	gripperbot_srv.request.z=z;
	gripperbot_srv.request.effector="gripper";

	if (loc->moveToOs.call(gripperbot_srv))
	{
		if(gripperbot_srv.response.success)
	    		std::cout << "Approach successfull " <<std::endl;
		else {
	    		std::cout << "Approach failed to " << x << " " << y << " " << z << std::endl;
			return false;
		}
	}
	else
	{
		ROS_ERROR("Failed to call service /robot_control/move_to_os");
		return false;
	}
	return true;
}

bool home()
{
robot_control::MoveToOS gripperbot_srv;
	static float home_x=0.33;
	static float home_y=0.00;
	static float home_z=0.34;

	gripperbot_srv.request.x=home_x;
	gripperbot_srv.request.y=home_y;
	gripperbot_srv.request.z=home_z;
	gripperbot_srv.request.effector="gripper";

	std::cout << "Moving Robot to Home" << std::endl;
	return moverobot(home_x,home_y,home_z); 

}

bool speechTurnOn() {
	re_speech_recognition::TurnOn req;
	if (!loc->speechTurnOn.call(req))
	{
		ROS_ERROR("Failed to call service speechTurnOn");
		return false;
	}
	if (req.response.success==0)
	{
		ROS_ERROR("Failed to call service speechTurnOn. Success is 0.");
		return false;
	}

	return true;
}

bool speechTurnOff() {
	re_speech_recognition::TurnOff req;
	if (!loc->speechTurnOff.call(req))
	{
		ROS_ERROR("Failed to call service speechTurnOff");
		return false;
	}

	return true;
}

bool speechLoadGrammar(std::string name) {
	re_speech_recognition::LoadGrammar req;
	req.request.name = name;
	if (!loc->speechLoadGrammar.call(req))
	{
		ROS_ERROR("Failed to call service speechLoadGrammar");
		return false;
	}
	if (!req.response.success)
	{
		ROS_ERROR("Failed to call service speechLoadGrammar. Success is 0.");
		return false;
	}
	return true;
}

void trackerCallback(const ros_tum_msgs::ActorVec& actorArr)
{

	geometry_msgs::Pose poseYellow;
	uint8_t yellowOk = 0;


    for (size_t i = 0; i < actorArr.ActorVec.size(); ++i)
    {
    	ROS_INFO("  ActorVec %d ", i);
    	ros_tum_msgs::Actor a = actorArr.ActorVec[i];
    	ROS_INFO("New Position: type = %s, property = %s, ", a.targetType.c_str(), a.targetProperty.c_str());
    	if (a.targetPoseVec.size()==0)
    		continue;
    	if (a.targetProperty.compare(loc->property)==0){
    	    poseYellow.position.x = (710.0-a.targetPoseVec[0].position.x)/1000.0;
    		poseYellow.position.y = (300.0-a.targetPoseVec[0].position.y)/1000.0;
    		poseYellow.position.z = a.targetPoseVec[0].position.z/1000.0;
    		yellowOk = 1;
	}
    }

    if (yellowOk == 0) {
		ROS_WARN("Yellow item not found. Retrying...");
		return;
    }


    std::cout << __LINE__ << std::endl;
	ROS_INFO("\tPose Yellow: %f|%f|%f", poseYellow.position.x,poseYellow.position.y,poseYellow.position.z );

    std::cout << __LINE__ << std::endl;
    robot_control::MoveToOS moveTo;

 
    moverobot(poseYellow.position.x-0.03, poseYellow.position.y, poseYellow.position.z+0.1);
    moverobot(poseYellow.position.x-0.03, poseYellow.position.y, poseYellow.position.z);

    closegripper();


    moverobot(poseYellow.position.x-0.03, poseYellow.position.y, poseYellow.position.z+0.1);

    home();

    speechTurnOn();

    loc->taken = true;


}

void speechCallback(const re_speech_recognition::RecognizedSpeech& speech) {

	ROS_INFO("[%3.3f] %s", speech.score, speech.speech.c_str());

	if( speech.speech.find("yellow") != string::npos) {
		strcpy(loc->property,"YELLOW");
		ROS_INFO("Taking yellow");
	} else if( speech.speech.find("green") != string::npos) {
		strcpy(loc->property,"GREEN");
		ROS_INFO("Taking green");
	} else {
		ROS_INFO("Green or Yellow not found");
		return;
	}

	if( !speechTurnOff())
		return;

	opengripper();

	//ROS_INFO("Taking %s", (char*)(loc->property));

	ros::Subscriber sub = loc->n.subscribe("/object_detector/objects_data", 1, trackerCallback);
	loc->taken = false;

	while(1) {
		ros::spinOnce();
		if(loc->taken)
			break;
	}



}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "speech_test");

	Ros_Localizer l;
	l.taken = false;
	loc = &l;

	ros::Subscriber sub = loc->n.subscribe("/re_speech_recognition/output", 1, speechCallback);

	loc->speechTurnOn = loc->n.serviceClient<re_speech_recognition::TurnOn>("/re_speech_recognition/turn_on");
	loc->speechTurnOff = loc->n.serviceClient<re_speech_recognition::TurnOff>("/re_speech_recognition/turn_off");
	loc->speechLoadGrammar = loc->n.serviceClient<re_speech_recognition::LoadGrammar>("/re_speech_recognition/load_grammar");



	loc->closeGripper = loc->n.serviceClient<gripper_control::CloseGripper>("/gripper_control/close_gripper");
	loc->openGripper = loc->n.serviceClient<gripper_control::OpenGripper>("/gripper_control/open_gripper");
	loc->moveToOs = loc->n.serviceClient<robot_control::MoveToOS>("/gripperbot_control/move_to_os");


	ROS_INFO("Loading grammar ...");

	if (!speechLoadGrammar("pro_grammar")) {
		return 1;
	}

	home();
	opengripper();

	if (!speechTurnOn()) {
		return 1;
	}

	ROS_INFO("Ready to Rumble");

	ros::spin();

	if (!speechTurnOff()) {
		return 1;
	}

	return 0;
}

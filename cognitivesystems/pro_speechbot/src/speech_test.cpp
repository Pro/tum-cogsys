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

#include <sstream>
#include <cstdlib>

struct Ros_Localizer {
	ros::NodeHandle n;
	ros::ServiceClient speechTurnOn;
	ros::ServiceClient speechTurnOff;
	ros::ServiceClient speechLoadGrammar;

};

struct Ros_Localizer *loc;

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

void speechCallback(const re_speech_recognition::RecognizedSpeech& speech) {

	ROS_INFO("[%3.3f] %s", speech.score, speech.speech.c_str());
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "speech_test");

	Ros_Localizer l;
	loc = &l;

	ros::Subscriber sub = loc->n.subscribe("/re_speech_recognition/output", 1, speechCallback);

	loc->speechTurnOn = loc->n.serviceClient<re_speech_recognition::TurnOn>("/re_speech_recognition/turn_on");
	loc->speechTurnOff = loc->n.serviceClient<re_speech_recognition::TurnOff>("/re_speech_recognition/turn_off");
	loc->speechLoadGrammar = loc->n.serviceClient<re_speech_recognition::LoadGrammar>("/re_speech_recognition/load_grammar");


	ROS_INFO("Loading grammar ...");

	if (!speechLoadGrammar("example")) {
		return 1;
	}
	if (!speechTurnOn()) {
		return 1;
	}

	ros::spin();

	if (!speechTurnOff()) {
		return 1;
	}

	return 0;
}

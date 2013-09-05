#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_tum_msgs/ActorVec.h"
#include "ros_tum_msgs/Actor.h"
#include "geometry_msgs/Pose.h"
#include "robot_control/MoveToOS.h"
#include "gripper_control/CloseGripper.h"
#include "gripper_control/OpenGripper.h"

#include <sstream>
#include <cstdlib>

struct Ros_Localizer {
	ros::NodeHandle n;
	ros::ServiceClient closeGripper;
	ros::ServiceClient openGripper;
	ros::ServiceClient moveToOs;


	uint8_t gripped;
	bool grasped;
	geometry_msgs::Pose oldRobotPose;
};

struct Ros_Localizer *loc;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void trackerCallback(const ros_tum_msgs::ActorVec& actorArr)
{
	if (loc->gripped) {
		ROS_INFO("Already gripped. Skipping...");
		return;
	}

	geometry_msgs::Pose poseYellow;
	geometry_msgs::Pose poseGreen;
	uint8_t yellowOk = 0;
	uint8_t greenOk = 0;


    for (size_t i = 0; i < actorArr.ActorVec.size(); ++i)
    {
    	ROS_INFO("  ActorVec %d ", i);
    	ros_tum_msgs::Actor a = actorArr.ActorVec[i];
    	ROS_INFO("New Position: type = %s, property = %s, ", a.targetType.c_str(), a.targetProperty.c_str());
    	if (a.targetPoseVec.size()==0)
    		continue;
    	if (a.targetProperty.compare("YELLOW")==0){
    		poseYellow.position.x = (710.0-a.targetPoseVec[0].position.x)/1000.0;
    		poseYellow.position.y = (300.0-a.targetPoseVec[0].position.y)/1000.0;
    		poseYellow.position.z = a.targetPoseVec[0].position.z/1000.0;
    		yellowOk = 1;
    	} else if (a.targetProperty.compare("GREEN")==0){
    		poseGreen.position.x = (710.0-a.targetPoseVec[0].position.x)/1000.0;
    		poseGreen.position.y = (300.0-a.targetPoseVec[0].position.y)/1000.0;
    		poseGreen.position.z = a.targetPoseVec[0].position.z/1000.0;
    		greenOk = 1;
    	}
    }

    if (yellowOk == 0) {
		ROS_WARN("Yellow item not found. Retrying...");
		return;
    }
    if (greenOk == 0) {
		ROS_WARN("Green item not found. Retrying...");
		return;
    }

    std::cout << __LINE__ << std::endl;
	ROS_INFO("\tPose Yellow: %f|%f|%f", poseYellow.position.x,poseYellow.position.y,poseYellow.position.z );
	ROS_INFO("\tPose Green: %f|%f|%f", poseGreen.position.x,poseGreen.position.y,poseGreen.position.z );

    std::cout << __LINE__ << std::endl;
    robot_control::MoveToOS moveTo;

    if(!loc->grasped)
    {

		moveTo.request.x = poseYellow.position.x-0.03;
		moveTo.request.y = poseYellow.position.y;
		moveTo.request.z = poseYellow.position.z+0.1;
		moveTo.request.effector = "gripper";

		loc->oldRobotPose.position.x = poseYellow.position.x;
		loc->oldRobotPose.position.y = poseYellow.position.y;
		loc->oldRobotPose.position.z = poseYellow.position.z;

		if (!loc->moveToOs.call(moveTo))
		{
			ROS_ERROR("Failed to call service MoveToOS over yellow");
			return;
		}
		if (moveTo.response.success == 0) {

			ROS_ERROR("MoveToOS over yellow failed, success is 0!");
			return;
		}

		moveTo.request.z = poseYellow.position.z;
		if (!loc->moveToOs.call(moveTo))
		{
			ROS_ERROR("Failed to call service MoveToOS at yellow");
			return;
		}
		if (moveTo.response.success == 0) {

			ROS_ERROR("MoveToOS at yellow failed, success is 0!");
			return;
		}

		gripper_control::CloseGripper grpClose;
		if (!loc->closeGripper.call(grpClose))
		{
			ROS_ERROR("Failed to call service close_gripper");
			return;
		}

		if (grpClose.response.success == 0) {

			ROS_ERROR("Gripper Close failed, success is 0!");
			return;
		}
		loc->grasped = true;

		return;
	}

    else
    {
    	moveTo.request.y = poseYellow.position.y;
    	moveTo.request.x = poseYellow.position.x-0.03;
		moveTo.request.z = poseYellow.position.z+0.1;
		moveTo.request.effector = "gripper";



		if (!loc->moveToOs.call(moveTo))
		{
			ROS_ERROR("Failed to call service MoveToOS over yellow gripped");
			return;
		}
		if (moveTo.response.success == 0) {

			ROS_ERROR("MoveToOS over yellow gripped failed, success is 0!");
			return;
		}

		/* Move to green box */
		//calculate offset after grapping

		double posOffsetX = poseYellow.position.x - loc->oldRobotPose.position.x;
		double posOffsetY = poseYellow.position.y - loc->oldRobotPose.position.y;

		double robotPosRadial = sqrt(pow(loc->oldRobotPose.position.x, 2) + pow(loc->oldRobotPose.position.y, 2) );
		double offsetRadial = sqrt(pow(posOffsetX, 2) + pow(posOffsetY, 2) );
		double offsetRadialProjected;
		if (offsetRadial < 0.001) {
			offsetRadialProjected = 0;
	    	ROS_INFO("Projected offset too small, using the default radial offset value");
		}
		else
		{
			offsetRadialProjected =
					offsetRadial*(posOffsetX*loc->oldRobotPose.position.x + posOffsetY*loc->oldRobotPose.position.y)/
					(offsetRadial*robotPosRadial);
			ROS_INFO("Projected offset (radial length): %.2f mm", offsetRadialProjected*1000.);
		}

		double correctionOffset = -0.010;

		double newRobotPosAngle = atan2(poseGreen.position.y, poseGreen.position.x);
		ROS_INFO("Green angle: %.2f deg", newRobotPosAngle*180./3.1415926);

		double newPosOffsetX = correctionOffset - offsetRadialProjected*cos(newRobotPosAngle);
		double newPosOffsetY = correctionOffset - offsetRadialProjected*sin(newRobotPosAngle);

		ROS_INFO("Used offset X: %.2f mm", newPosOffsetX*1000);
		ROS_INFO("Used offset Y: %.2f mm", newPosOffsetY*1000);

		moveTo.request.x = poseGreen.position.x - newPosOffsetX;
		moveTo.request.y = poseGreen.position.y - newPosOffsetY;
		moveTo.request.z = poseGreen.position.z+0.1;
		if (!loc->moveToOs.call(moveTo))
		{
			ROS_ERROR("Failed to call service MoveToOS over green");
			return;
		}
		if (moveTo.response.success == 0) {

			ROS_ERROR("MoveToOS over green failed, success is 0!");
			return;
		}

		moveTo.request.x = poseGreen.position.x-0.03-posOffsetX;
		moveTo.request.y = poseGreen.position.y-posOffsetY;
		moveTo.request.z = poseGreen.position.z+0.027;
		moveTo.request.effector = "gripper";
		if (!loc->moveToOs.call(moveTo))
		{
			ROS_ERROR("Failed to call service MoveToOS at green");
			return;
		}
		if (moveTo.response.success == 0) {

			ROS_ERROR("MoveToOS at green failed, success is 0!");
			return;
		}

		gripper_control::OpenGripper grpOpen;
		if (!loc->openGripper.call(grpOpen))
		{
			ROS_ERROR("Failed to call service open gripper");
			return;
		}

		if (grpOpen.response.success == 0) {

			ROS_ERROR("Gripper open failed, success is 0!");
			return;
		}


		moveTo.request.x = 0.3;
		moveTo.request.y = 0;
		moveTo.request.z = 0.3;
		if (!loc->moveToOs.call(moveTo))
		{
			ROS_ERROR("Failed to call service MoveToOS end");
			return;
		}
		if (moveTo.response.success == 0) {

			ROS_ERROR("MoveToOS end failed, success is 0!");
			return;
		}

		ROS_INFO("Gripping successfully finished!");
		loc->gripped = 1;

    }


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cylinder_grip");

	Ros_Localizer l;
	l.gripped = 0;
	l.grasped = false;
	loc = &l;

	ros::Subscriber sub = loc->n.subscribe("/object_detector/objects_data", 1, trackerCallback);

	loc->closeGripper = loc->n.serviceClient<gripper_control::CloseGripper>("/gripper_control/close_gripper");
	loc->openGripper = loc->n.serviceClient<gripper_control::OpenGripper>("/gripper_control/open_gripper");
	loc->moveToOs = loc->n.serviceClient<robot_control::MoveToOS>("/gripperbot_control/move_to_os");


	ROS_INFO("Initializing robot position...");

	gripper_control::OpenGripper grpOpen;
	if (!loc->openGripper.call(grpOpen))
	{
		ROS_ERROR("Failed to call service open gripper");
		return 1;
	}

	if (grpOpen.response.success == 0) {

		ROS_ERROR("Gripper open failed, success is 0!");
		return 1;
	}

	robot_control::MoveToOS moveTo;
	moveTo.request.x = 0.3;
	moveTo.request.y = 0;
	moveTo.request.z = 0.3;
	moveTo.request.effector = "gripper";
	if (!loc->moveToOs.call(moveTo))
	{
		ROS_ERROR("Failed to call service MoveToOS init");
		return 1;
	}
	if (moveTo.response.success == 0) {

		ROS_ERROR("MoveToOS init failed, success is 0!");
		return 1;
	}


	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
	ros::spin();

	return 0;
}

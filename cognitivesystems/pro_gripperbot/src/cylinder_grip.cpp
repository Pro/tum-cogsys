#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_tum_msgs/ActorVec.h"
#include "ros_tum_msgs/Actor.h"
#include "geometry_msgs/Pose.h"
#include "robot_control/MoveToOS.h"
#include "gripper_control/CloseGripper.h"

#include <sstream>
#include <cstdlib>

struct Ros_Localizer {
	ros::NodeHandle n;
	ros::ServiceClient closeGripper;
	ros::ServiceClient moveToOs;

};

struct Ros_Localizer *loc;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void trackerCallback(const ros_tum_msgs::ActorVec& actorArr)
{
    for (size_t i = 0; i < actorArr.ActorVec.size(); ++i)
    {
    	ROS_INFO("  ActorVec %d ", i);
    	ros_tum_msgs::Actor a = actorArr.ActorVec[i];
    	ROS_INFO("New Position: type = %s, property = %s, ", a.targetType.c_str(), a.targetProperty.c_str());
    	if (a.targetProperty.compare("YELLOW")==0){
			geometry_msgs::Pose p = a.targetPoseVec[i];
			ROS_INFO("\tPose[%d]: %f|%f|%f", i,p.position.x,p.position.y,p.position.z );

			robot_control::MoveToOS moveTo;
			moveTo.request.x = (710.0-p.position.x)/1000.0;
			moveTo.request.y = (300.0-p.position.y)/1000.0;
			moveTo.request.z = p.position.z/1000.0+0.1;
			moveTo.request.effector = "gripper";
			if (!loc->moveToOs.call(moveTo))
			{
				ROS_ERROR("Failed to call service MoveToOS 1");
				return;
			}
			if (moveTo.response.success == 0) {

				ROS_ERROR("MoveToOS 1 failed, success is 0!");
				return;
			}

			moveTo.request.z = p.position.z/1000.0;
			if (!loc->moveToOs.call(moveTo))
			{
				ROS_ERROR("Failed to call service MoveToOS 2");
				return;
			}
			if (moveTo.response.success == 0) {

				ROS_ERROR("MoveToOS 2 failed, success is 0!");
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

			ROS_INFO("Gripping successfully finished!");

    	}
    }


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cylinder_grip");

	Ros_Localizer l;
	loc = &l;

	ros::Subscriber sub = loc->n.subscribe("/object_detector/objects_data", 1000, trackerCallback);

	loc->closeGripper = loc->n.serviceClient<gripper_control::CloseGripper>("/gripper_control/close_gripper");
	loc->moveToOs = loc->n.serviceClient<robot_control::MoveToOS>("/gripperbot_control/move_to_os");

	ROS_INFO("Initializing robot position...");
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

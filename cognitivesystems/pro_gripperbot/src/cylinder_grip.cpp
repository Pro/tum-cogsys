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
    	ROS_WARN("  ActorVec %d ", i);
    	ros_tum_msgs::Actor a = actorArr.ActorVec[i];
    	ROS_WARN("New Position: type = %s, property = %s, ", a.targetType.c_str(), a.targetProperty.c_str());
    	if (a.targetProperty.compare("YELLOW")==0){
			geometry_msgs::Pose p = a.targetPoseVec[i];
			ROS_WARN("\tPose[%d]: %f|%f|%f", i,p.position.x,p.position.y,p.position.z );

			robot_control::MoveToOS moveTo;
			moveTo.request.x = p.position.x;
			moveTo.request.y = p.position.y;
			moveTo.request.z = p.position.z;
			moveTo.request.effector = "gripper";
			if (loc->moveToOs.call(moveTo))
				ROS_INFO("MoveToOS Success: %ld", (long int)moveTo.response.success);
			else
			{
				ROS_ERROR("Failed to call service MoveToOS");
				return;
			}


			gripper_control::CloseGripper srv;
			if (loc->closeGripper.call(srv))
			{
				ROS_INFO("CloseGripper: %ld", (long int)srv.response.success);
			}
			else
			{
				ROS_ERROR("Failed to call service close_gripper");
				return;
			}

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

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
	ros::spin();

	return 0;
}

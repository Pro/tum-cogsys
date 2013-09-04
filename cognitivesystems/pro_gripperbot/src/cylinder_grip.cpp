#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gripper_control/CloseGripper.h"

#include <sstream>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cylinder_grip");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gripper_control::CloseGripper>("close_gripper");
  gripper_control::CloseGripper srv;
  if (client.call(srv))
  {
    ROS_INFO("Success: %ld", (long int)srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service close_gripper");
    return 1;
  }

  return 0;
}

#include <ros/ros.h>

#include "uvc_opt_flow.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "uvc_ros_flow");

  OptFlow opt_flow;

  ros::spin();
  return 0;
}

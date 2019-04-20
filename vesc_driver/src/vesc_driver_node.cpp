#include <ros/ros.h>

#include "vesc_driver/vesc_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vesc_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  vesc_driver::VescDriver vesc_driver(nh, private_nh);

  bool using_sim_time = false;
  if (!nh.getParam("/use_sim_time", using_sim_time)) {
    fprintf(stderr, "ERROR: Unable to check for sim time!\n");
    exit(1);
  }
  if (using_sim_time) {
    fprintf(stderr, "ERROR: Cannot run driver when /using_sim_time is true!\n");
    exit(1);
  }
  ros::spin();

  return 0;
}

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <ros/ros.h>

#include "wp_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnss_remap");
  way_point::Gnss_Zmap Gnss_Zmap;
  Gnss_Zmap.run();

  return 0;
}

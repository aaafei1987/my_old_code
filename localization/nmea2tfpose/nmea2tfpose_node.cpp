#include <ros/ros.h>

#include "nmea2tfpose_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea2tfpose");
  gnss_localizer::Nmea2TFPoseNode ntpn;
  way_point::Gnss_Zmap wpn;
  ntpn.run();

  return 0;
}


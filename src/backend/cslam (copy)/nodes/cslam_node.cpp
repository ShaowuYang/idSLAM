#include <ros/ros.h>

#include "CSLAMNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cslam_node");

  ros::NodeHandle n;
  cslam::CSLAMNode node(n);

  ros::spin();

  return 0;
}


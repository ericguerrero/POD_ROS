#include <ros/ros.h>
#include "podnode.h"

int main(int argc, char **argv) {
  ros::init(argc,argv,"pod");
  if (argc!=3) {
    ROS_ERROR("Arguments: Path/to/config overlayed (0/1)");
    return EXIT_FAILURE;
  }
  ros::NodeHandle nh;
  PODNode theNode(nh,string(argv[1]),argv[2][0]=='1');
  theNode.start();
  return EXIT_SUCCESS;
}
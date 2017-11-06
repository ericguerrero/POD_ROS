// Posidonia Oceanica Detector (POD) ROS wrapper.
// Usage example:
//  ros::init(argc,argv,"pod");
//  ros::NodeHandle nh;
//  PODNode theNode(nh,"path/to/config/");
//  theNode.start();
// Author   : A. Burguera.
// Creation : 8-March-2017

#ifndef _PODNODE_H_
#define _PODNODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>


#include "podefines.h"
#include "podescriber.h"
#include "podetector.h"

using namespace cv;
using namespace std;

class PODNode {
private:
  PODetector *_theDetector;         // The detector himself!
  ros::NodeHandle _nh;              // Node handler copy.
  image_transport::ImageTransport _it;  // Required for image messages
  image_transport::Subscriber _sub; // Subscriber to input images
  ros::Publisher _pub_desc;  // Publishes descriptor matrix
  image_transport::Publisher _pub_class;  // Publishes classified images
  image_transport::Publisher _pub_image;  // Publishes classified images
  int _overlay;                     // Output selector
  // The image message callback.
  void processImage(const sensor_msgs::ImageConstPtr& msg);
public:
  // The constructor. Requires the path to the config dir.
  PODNode(const ros::NodeHandle& nh,string configPath,int overlay);
  // Frees dynamically allocated memory.
  ~PODNode();
  // Just makes the node spin.
  void start();
};

#endif
#include "podnode.h"

PODNode::PODNode(const ros::NodeHandle& nh,string configPath,int overlay):_nh(nh),_it(nh),_overlay(overlay) {
  //Initialize detector
  _theDetector=new PODetector(configPath);

  //Set subscriber; input image
  _sub=_it.subscribe("image_in", 1, &PODNode::processImage,this);

  //Set publishers; descriptors, overlayed classification, and binary classification.
  _pub_desc=_nh.advertise<std_msgs::Float64MultiArray>("descriptors", 1000);
  _pub_image=_it.advertise("image_out", 1);
  _pub_class=_it.advertise("classification", 1);
}

PODNode::~PODNode() {
  if (_theDetector) delete _theDetector;
}

void PODNode::processImage(const sensor_msgs::ImageConstPtr& msg) {
  Mat outDes;
  Mat outMat;
  Mat outImage;
  std_msgs::Header header;

  // Set image and classify patches
  _theDetector->set_image(cv_bridge::toCvShare(msg, "bgr8")->image);
  _theDetector->classify_image();

  // Get patch descriptors, overlayed classification, and binary classification
  outDes=_theDetector->get_descriptor();
  outMat=_theDetector->get_binary_classification();
  outImage=_theDetector->get_graphic_classification();

  // Publish
  _pub_desc.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", outDes).toImageMsg());
  _pub_image.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", outImage).toImageMsg());
  _pub_class.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", outMat).toImageMsg());
}

void PODNode::start() {
  ros::spin();
}
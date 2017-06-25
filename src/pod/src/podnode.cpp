#include "podnode.h"

PODNode::PODNode(const ros::NodeHandle& nh,string configPath,int overlay):_nh(nh),_it(nh),_overlay(overlay) {
  _theDetector=new PODetector(configPath);
  _sub=_it.subscribe("image_in", 1, &PODNode::processImage,this);
  _pub=_it.advertise("image_out", 1);
}

PODNode::~PODNode() {
  if (_theDetector) delete _theDetector;
}

void PODNode::processImage(const sensor_msgs::ImageConstPtr& msg) {
  Mat outImage;
  _theDetector->set_image(cv_bridge::toCvShare(msg, "bgr8")->image);
  _theDetector->classify_image();
  _theRefiner.set_data(_theDetector->get_image(),_theDetector->get_classification());
  _theRefiner.process();
  outImage=_overlay?_theRefiner.get_refined_overlayed():_theRefiner.get_refined_scaled();
  _pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", outImage).toImageMsg());
}

void PODNode::start() {
  ros::spin();
}
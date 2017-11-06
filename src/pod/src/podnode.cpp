#include "podnode.h"

PODNode::PODNode(const ros::NodeHandle& nh,string configPath,int overlay):_nh(nh),_it(nh),_overlay(overlay) {
  //Initialize detector
  _theDetector=new PODetector(configPath);

  //Set subscriber; input image
  _sub=_it.subscribe("image_in", 1, &PODNode::processImage,this);

  //Set publishers; descriptors, overlayed classification, and binary classification.
  _pub_desc=_nh.advertise<std_msgs::Float64MultiArray>("descriptors", 1);
  _pub_image=_it.advertise("image_out", 1);
  _pub_class=_it.advertise("classification", 1);
}

PODNode::~PODNode() {
  if (_theDetector) delete _theDetector;
}

void PODNode::processImage(const sensor_msgs::ImageConstPtr& msg) {
  float outDes;
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

  cout << "\n";
  cout << outDes;
  cout << "\n";

  // Build MultiArray message for diescriptors
  //std_msgs::Float64MultiArray msg_desc;
  //msg_desc.layout.dim[0].label = "patch_row";
  //msg_desc.layout.dim[0].size =  _POD_NPATCH_ROW_;
  //msg_desc.layout.dim[0].stride =  _POD_NPATCH_ROW_*_POD_NPATCH_COL_*_POD_DESCR_TYPES_*_POD_IMAGE_NCHAN_*_POD_GABOR_SCALES_*_POD_GABOR_ORIENT_;
  //msg_desc.layout.dim[1].label = "patch_col";
  //msg_desc.layout.dim[1].size =  _POD_NPATCH_COL_;
  //msg_desc.layout.dim[1].stride =  _POD_NPATCH_COL_*_POD_DESCR_TYPES_*_POD_IMAGE_NCHAN_*_POD_GABOR_SCALES_*_POD_GABOR_ORIENT_;
  //msg_desc.layout.dim[2].label = "descr_types";
  //msg_desc.layout.dim[2].size =  _POD_DESCR_TYPES_;
  //msg_desc.layout.dim[2].stride =  _POD_DESCR_TYPES_*_POD_IMAGE_NCHAN_*_POD_GABOR_SCALES_*_POD_GABOR_ORIENT_;
  //msg_desc.layout.dim[3].label = "descr_nchan";
  //msg_desc.layout.dim[3].size =  _POD_IMAGE_NCHAN_;
  //msg_desc.layout.dim[3].stride =  _POD_IMAGE_NCHAN_*_POD_GABOR_SCALES_*_POD_GABOR_ORIENT_;
  //msg_desc.layout.dim[4].label = "gabor_scales";
  //msg_desc.layout.dim[4].size =  _POD_GABOR_SCALES_;
  //msg_desc.layout.dim[4].stride =  _POD_GABOR_SCALES_*_POD_GABOR_ORIENT_;
  //msg_desc.layout.dim[5].label = "gabor_orient";
  //msg_desc.layout.dim[5].size =  _POD_GABOR_ORIENT_;
  //msg_desc.layout.dim[5].stride =  _POD_GABOR_ORIENT_;
  //msg_desc.data = *outDes;
//
  //// Publish
  ////_pub_desc.publish(msg_desc);
  _pub_image.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", outImage).toImageMsg());
  _pub_class.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", outMat).toImageMsg());
}

void PODNode::start() {
  ros::spin();
}
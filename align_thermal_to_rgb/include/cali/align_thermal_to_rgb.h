#ifndef _ALIGN_DEPTH_TO_RGB_H
#define _ALIGN_DEPTH_TO_RGB_H

#include <cassert>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include "../src/helper.hpp"

class align_thermal_to_rgb{
 public:
  align_thermal_to_rgb();
  align_thermal_to_rgb(std::string);
  align_thermal_to_rgb(cv::Mat, cv::Mat, tf::Transform);
 private:
  bool has_camera_model;
  bool has_rgb_intrinsic;
  bool has_rgb_image;
  bool has_depth_image;
  bool has_thermal_image;
  const int kernel_size = 7;
  cv::Mat intrinsic_;
  cv::Mat distortion_;
  cv::Mat rgb_image_;
  cv::Mat depth_image_;
  cv::Mat thermal_image_;
  Intrinsic rgb_intrinsic_;
  tf::Transform extrinsic_;
 public:
  void set_camera_model(cv::Mat, cv::Mat, tf::Transform);
  void set_rgb_image(cv::Mat img) {rgb_image_ = img.clone(); has_rgb_image = true;}
  void set_depth_image(cv::Mat img) {if(img.type()==CV_16UC1) depth_image_ = img.clone(); has_depth_image = true;}
  void set_thermal_image(cv::Mat img) {thermal_image_ = img.clone(); has_thermal_image = true;}
  void set_rgb_intrinsic(Intrinsic);
  void set_rgb_intrinsic(std::vector<double>);
  void set_rgb_intrinsic(double, double, double, double);
  void show_model(void);
  void align(cv::Mat&);
};
#endif

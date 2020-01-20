#include "cali/align_thermal_to_rgb.h"

align_thermal_to_rgb::align_thermal_to_rgb(): 
  has_camera_model(false), 
  has_rgb_intrinsic(false),
  has_rgb_image(false),
  has_depth_image(false), 
  has_thermal_image(false)
  {}
  
align_thermal_to_rgb::align_thermal_to_rgb(std::string in_file):
  has_rgb_intrinsic(false),
  has_rgb_image(false),
  has_depth_image(false),
  has_thermal_image(false)
{
  read_file(in_file, intrinsic_, distortion_, extrinsic_);
  has_camera_model = true;
}

align_thermal_to_rgb::align_thermal_to_rgb(cv::Mat intrinsic, cv::Mat distortion, tf::Transform extrinsic):
  has_rgb_intrinsic(false),
  has_rgb_image(false),
  has_depth_image(false),
  has_thermal_image(false)
{
  set_camera_model(intrinsic, distortion, extrinsic);
}

void align_thermal_to_rgb::set_camera_model(cv::Mat intrinsic, cv::Mat distortion, tf::Transform extrinsic){
  //if(intrinsic.size==cv::Size(3, 3)) 
    intrinsic_ = intrinsic.clone();
  //if(distortion.size==cv::Size(1, 5)) 
    distortion_ = distortion.clone();
  extrinsic_ = tf::Transform(extrinsic);
  has_camera_model = true;
}

void align_thermal_to_rgb::set_rgb_intrinsic(Intrinsic intrinsic){
  rgb_intrinsic_.fx = intrinsic.fx;
  rgb_intrinsic_.fy = intrinsic.fy;
  rgb_intrinsic_.cx = intrinsic.cx;
  rgb_intrinsic_.cy = intrinsic.cy;
  has_rgb_intrinsic = true;
}

void align_thermal_to_rgb::set_rgb_intrinsic(std::vector<double> vec){
  assert(vec.size()==4);
  rgb_intrinsic_.fx = vec[0];
  rgb_intrinsic_.fy = vec[1];
  rgb_intrinsic_.cx = vec[2];
  rgb_intrinsic_.cy = vec[3];
  has_rgb_intrinsic = true;
}

void align_thermal_to_rgb::set_rgb_intrinsic(double fx, double fy, double cx, double cy){
  rgb_intrinsic_.fx = fx;
  rgb_intrinsic_.fy = fy;
  rgb_intrinsic_.cx = cx;
  rgb_intrinsic_.cy = cy;
  has_rgb_intrinsic = true;
}

void align_thermal_to_rgb::align(cv::Mat &dst){
  if(has_camera_model && has_rgb_intrinsic && has_rgb_image && has_depth_image && has_thermal_image){
    dst = cv::Mat::zeros(rgb_image_.size(), thermal_image_.type());
    for(int i=0; i<rgb_image_.cols; ++i){
      for(int j=0; j<rgb_image_.rows; ++j){
        tf::Vector3 rgb_xyz = position2tf(getPosition(i, j, rgb_intrinsic_, depth_image_, true, kernel_size));
        if(rgb_xyz.getZ()==0.0) continue; // Ignore if z equals to zero
        tf::Vector3 thermal_xyz = extrinsic_*rgb_xyz;
        int u, v;
        project2pixel_plane(thermal_xyz, intrinsic_, distortion_, u, v);
        if(u>=0 && u<thermal_image_.cols && v>=0 && v<thermal_image_.rows){
          //std::cout << "(" << i << ", " << j << ") -> (" << u << ", " << v << ")\n";
          dst.at<unsigned char>(j, i) = thermal_image_.at<unsigned char>(v, u);
        }
      }
    }
    has_rgb_image = false; has_depth_image = false; has_thermal_image = false; // Processed
  }else{
    std::cout << "\033[1;33mNot ready, ignore request...\033[0m\n";
  }
}

void align_thermal_to_rgb::show_model(void){
  std::cout << "RGB intrinsic: \n";
  std::cout << rgb_intrinsic_.fx << " " 
            << rgb_intrinsic_.fy << " "
            << rgb_intrinsic_.cx << " "
            << rgb_intrinsic_.cy << "\n\n";
  std::cout << "Thermal intrinsic: \n";
  std::cout << intrinsic_.type() << "\n";
  std::cout << intrinsic_ << "\n\n";
  std::cout << "Thermal distortion: \n";
  std::cout << distortion_.type() << "\n";
  std::cout << distortion_ << "\n\n";
  std::cout << "Extrinsic: \n";
  std::cout << extrinsic_.getRotation().getX() << " "
            << extrinsic_.getRotation().getY() << " "
            << extrinsic_.getRotation().getZ() << " "
            << extrinsic_.getRotation().getW() << " "
            << extrinsic_.getOrigin().getX() << " "
            << extrinsic_.getOrigin().getY() << " "
            << extrinsic_.getOrigin().getZ() << "\n\n";
}

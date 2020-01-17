#include <fstream>
#include <string>
#include <cassert>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tf/tf.h>

typedef struct Intrinsic{
  double fx;
  double fy;
  double cx;
  double cy;
}Intrinsic;

typedef struct Position{
  double x;
  double y;
  double z;
}Position;

/*
 *  Print functions
 */

void show_help(void){
  std::cout << "\033[1;31mNot enough argument given!\n\033[0m";
  std::cout << "\033[1;31nPlease provide image paths of rgb, depth and thermal (`string`), and then the kernel size (`int`)\033[0m\n";
}

void print_position(int idx, Position pos){
  std::cout << idx << ": " << "(" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
}

/*
 *  Conversion functions
 */

cv::Point3f position2cvPoint3f(Position pos){
  cv::Point3f cvPoint3f;
  cvPoint3f.x = pos.x;
  cvPoint3f.y = pos.y;
  cvPoint3f.z = pos.z;
  return cvPoint3f;
}

tf::Vector3 position2tf(Position pos){
  tf::Vector3 res(pos.x, pos.y, pos.z);
  return res;
}

tf::Transform RT2tf(cv::Mat rvec, cv::Mat tvec){
  tf::Matrix3x3 rot_mat;
  cv::Mat rot_mat_cv(cv::Size(3, 3), CV_64F); cv::Rodrigues(rvec, rot_mat_cv);
  for(int i=0; i<3; ++i){
    for(int j=0; j<3; ++j){
      rot_mat[i][j] = rot_mat_cv.at<double>(i, j);
    }
  }
  tf::Quaternion quat; rot_mat.getRotation(quat);
  tf::Vector3 trans(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
  tf::Transform homo(quat, trans);
  return homo;
}

// Use mean filter to compensate zero-depth pixel
double mean_filter(double u, double v, cv::Mat depth_img, int kernel_size){
  double mean_z = 0.0;
  int cnt = 0;
  for(int i=-kernel_size/2; i<=kernel_size/2; ++i){
    for(int j=-kernel_size/2; j<=kernel_size/2; ++j){
      if(v+i<0 || v+i>depth_img.rows || u+j<0 || u+j>depth_img.cols) continue; // Check if exceed the boarder
      if(depth_img.at<unsigned short>(v+i, u+j)!=0){
        mean_z += depth_img.at<unsigned short>(v+i, u+j); cnt += 1;
      }
    }
  }
  return mean_z/(double)cnt;
}

// Use pixel coordinate (u, v) and depth (z) to compute coordinate w.r.t. camera frame by pinhole model
Position getPosition(double u, double v, Intrinsic intrinsic, cv::Mat depth_img, bool enable_mean_filter, int kernel_size){
  Position pos;
  double z = depth_img.at<unsigned short>(v, u);
  if(z==0){ 
    if(enable_mean_filter){
      z = mean_filter(u, v, depth_img, kernel_size);
    }
  }
  pos.z = z/1000.; // mm to meter
  pos.x = (u-intrinsic.cx)*pos.z/intrinsic.fx; // x = (u-cx)*z/fx
  pos.y = (v-intrinsic.cy)*pos.z/intrinsic.fy; // y = (v-cy)*z/fy
  return pos;
}

void project2pixel_plane(tf::Vector3 xyz, cv::Mat intrinsic, cv::Mat distort, int &u, int &v){
  // Ref: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#calibratecamera
  double x_prime = xyz.getX()/xyz.getZ();
  double y_prime = xyz.getY()/xyz.getZ();
  double r_square = x_prime*x_prime + y_prime*y_prime;
  double fx = intrinsic.at<double>(0, 0);
  double fy = intrinsic.at<double>(1, 1);
  double cx = intrinsic.at<double>(0, 2);
  double cy = intrinsic.at<double>(1, 2);
  double k1 = distort.at<double>(0, 0);
  double k2 = distort.at<double>(0, 1);
  double p1 = distort.at<double>(0, 2);
  double p2 = distort.at<double>(0, 3);
  double k3 = distort.at<double>(0, 4);
  double factor = 1+k1*r_square+k2*r_square*r_square+k3*r_square*r_square*r_square;
  double x_double_prime = x_prime*factor+2*p1*x_prime*y_prime+p2*(r_square+2*x_prime*x_prime);
  double y_double_prime = y_prime*factor+p1*(r_square+2*y_prime*y_prime)+2*p2*x_prime*y_prime;
  u = fx*x_double_prime + cx;
  v = fy*y_double_prime + cy;
}

/*
 *  I/O
 */ 
void write_file(cv::Mat intrinsic, cv::Mat distortion, tf::Transform extrinsic){
  std::fstream fs;
  fs.open("camera_model.txt", std::fstream::out | std::fstream::trunc);
  tf::Quaternion quat = extrinsic.getRotation();
  fs << "intrinsic: [" << intrinsic.at<double>(0, 0) << ", "   // fx
                       << intrinsic.at<double>(1, 1) << ", "   // fy
                       << intrinsic.at<double>(0, 2) << ", "   // cx
                       << intrinsic.at<double>(1, 2) << "]\n";  // cy
  fs << "distortion: [" << distortion.at<double>(0, 0) << ", "   // k1
                        << distortion.at<double>(0, 1) << ", "   // k2
                        << distortion.at<double>(0, 2) << ", "   // p1
                        << distortion.at<double>(0, 3) << ", "   // p2
                        << distortion.at<double>(0, 4) << "]\n"; // k3
  fs << "quaternion: [" << quat.getX() << ", "   // qx
                        << quat.getY() << ", "   // qy
                        << quat.getZ() << ", "   // qz
                        << quat.getW() << "]\n"; // qw
  tf::Vector3 trans = extrinsic.getOrigin();
  fs << "origin: [" << trans.getX() << ", "    // x
                    << trans.getY() << ", "    // y
                    << trans.getZ() << "]\n";  // z
  fs.close();
}

bool parse_line(std::string line, std::vector<double> &vec){
  std::size_t colon = line.find(":");
  if(colon==std::string::npos) return false;
  std::string key = line.substr(0, colon);
  if(key!="quaternion" &&
     key!="origin" &&
     key!="intrinsic" &&
     key!="distortion") 
    return false;
  std::string data_str = line.substr(colon+2, line.length()-colon-1);
  std::size_t comma = data_str.find(",");
  std::string component_str;
  int start_idx = 1;
  while(comma!=std::string::npos){
    component_str = data_str.substr(start_idx, comma-start_idx);
    vec.push_back(atof(component_str.c_str()));
    start_idx = comma+2;
    comma = data_str.find(",", comma+2);
  }
  component_str = data_str.substr(start_idx, data_str.length()-start_idx); vec.push_back(atof(component_str.c_str()));
  if((key=="quaternion" && vec.size()!=4) || 
     (key=="origin" && vec.size()!=3) ||
     (key=="intrinsic" && vec.size()!=4) || 
     (key=="distortion" && vec.size()!=5)) 
    return false;
  return true;
}

void read_file(std::string in_file, cv::Mat &intrinsic, cv::Mat &distortion, tf::Transform &extrinsic){
  std::fstream fs;
  std::string intrinsic_line, distortion_line, quat_line, trans_line;
  fs.open(in_file, std::fstream::in);
  assert(fs.is_open());
  std::vector<double> intrinsic_vec, distortion_vec, quat_vec, trans_vec;
  std::getline(fs, intrinsic_line); std::getline(fs, distortion_line);
  std::getline(fs, quat_line); std::getline(fs, trans_line);
  assert(parse_line(intrinsic_line, intrinsic_vec));
  assert(parse_line(distortion_line, distortion_vec));
  assert(parse_line(quat_line, quat_vec));
  assert(parse_line(trans_line, trans_vec));
  intrinsic = (cv::Mat1f(3, 3) << intrinsic_vec[0], 0.0,  intrinsic_vec[2] \
                               , 0.0, intrinsic_vec[1], intrinsic_vec[3] \
                               , 0.0, 0.0, 1.0);
  distortion = (cv::Mat1f(1, 5) << distortion_vec[0] 
                                , distortion_vec[1] 
                                , distortion_vec[2]  
                                , distortion_vec[3]
                                , distortion_vec[4]);
  intrinsic.convertTo(intrinsic, CV_64F);
  distortion.convertTo(distortion, CV_64F);
  tf::Quaternion quat(quat_vec[0], quat_vec[1], quat_vec[2], quat_vec[3]); // qx, qy, qz, qw
  tf::Vector3 trans(trans_vec[0], trans_vec[1], trans_vec[2]); // x, y, z
  extrinsic = tf::Transform(quat, trans);
}

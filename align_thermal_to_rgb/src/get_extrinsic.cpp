#include <iostream>
#include "libcbdetect/boards_from_cornres.h"
#include "libcbdetect/boards_from_cornres.h"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_boards.h"
#include "libcbdetect/plot_corners.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tf/tf.h>
#include "helper.hpp"

void plot_corners_idx(cv::Mat image, cbdetect::Corner corners, cbdetect::Board board){
  int idx_cnt = 0;
  cv::Mat image_draw; image.copyTo(image_draw);
  for(int i=0; i<board.idx.size(); ++i){
    for(int j=0; j<board.idx[i].size(); ++j){
      if(board.idx[i][j]<0) continue;
      cv::putText(image_draw, std::to_string(idx_cnt), corners.p[board.idx[i][j]], cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(255, 0, 0), 1);
      idx_cnt++;
    }
  }
  cv::namedWindow("corners", cv::WINDOW_AUTOSIZE);
  cv::imshow("corners", image_draw);
  cv::waitKey();
}

int main(int argc, char** argv)
{
  // ./[program_name] [rgb_intrinsic_path] [thermal_intrinsic_path] [rgb_image_path] [depth_image_path] [thermal_image_path] [kernel_size]
  if(argc!=7) {
    show_help();
    exit(EXIT_FAILURE);
  }
  Intrinsic intrinsic;
  parse_rgb_intrinsic(argv[1], intrinsic);
  cv::Mat thermal_intrinsic, thermal_distortion;
  parse_thermal_intrinsic(argv[2], thermal_intrinsic, thermal_distortion);
  // Read input arguments
  cv::Mat rgb     = cv::imread(argv[3], CV_LOAD_IMAGE_COLOR);
  cv::Mat depth   = cv::imread(argv[4], CV_LOAD_IMAGE_ANYDEPTH);
  cv::Mat thermal = cv::imread(argv[5], CV_LOAD_IMAGE_GRAYSCALE);
  if(rgb.empty() || depth.empty() || thermal.empty()){
    std::cout << "\033[1;33mInvalid given image path, stop processing\033[0m\n";
    exit(EXIT_FAILURE);
  }
  int kernel_size = atoi(argv[6]);
  std::vector<Position> position_vec; // Corner position in RGB image frame placeholder
  /*
   *  Detect corners in RGB image
   */
  cbdetect::Corner corners;
  std::vector<cbdetect::Board> boards;
  cbdetect::Params params;
  #ifndef DEBUG
  params.show_processing = false;
  #endif
  params.corner_type = cbdetect::SaddlePoint;
  cbdetect::find_corners(rgb, corners, params);
  cbdetect::boards_from_corners(rgb, corners, boards, params);
  if(boards.size()==0){
    std::cout << "\033[1;33mNo board detected, make sure that chessboard pattern in both RGB and thermal images\033[0m\n";
    exit(EXIT_FAILURE);
  }  
  #ifdef DEBUG
  cbdetect::plot_corners(image_in, corners);
  cbdetect::plot_boards(image_in, corners, boards, params);
  plot_corners_idx(rgb, corners, boards[0]);
  #endif
  int internal_corners_cnt = 0;
  // Get corner position by pixels, intrinsic and depth information
  for(int i=0; i<boards[0].idx.size(); ++i){
    for(int j=0; j<boards[0].idx[i].size(); ++j){
      if(boards[0].idx[i][j]<0) continue;
      position_vec.push_back(getPosition(corners.p[boards[0].idx[i][j]].x, corners.p[boards[0].idx[i][j]].y, intrinsic, depth, true, kernel_size));
      ++internal_corners_cnt;
    }
  }
  std::cout << "Detect \033[1;33m" << internal_corners_cnt << "\033[0m internal corners\n";
  #ifdef DEBUG
  for(int i=0; i<internal_corners_cnt; ++i)
    print_position(i, position_vec[i]);
  #endif
  /*
   *  Detect board in thermal image
   */
  cbdetect::Corner corners_thermal;
  std::vector<cbdetect::Board>boards_thermal;
  cbdetect::find_corners(thermal, corners_thermal, params);
  cbdetect::boards_from_corners(thermal, corners_thermal, boards_thermal, params);
  if(boards_thermal.size()==0){
    std::cout << "\033[1;33mNo board detected, make sure that chessboard pattern in both RGB and thermal images\033[0m\n";
    exit(EXIT_FAILURE);
  }
  #ifdef DEBUG
  plot_corners_idx(thermal, corners_thermal, boards_thermal[0]);
  #endif
  // Call cv::cameraCalibration to solve extrinsic and intrinsic
  std::vector<cv::Point3f> object_position;
  std::vector<cv::Point2f> image_pixels;
  std::vector<std::vector<cv::Point3f>> obj; 
  std::vector<std::vector<cv::Point2f>> img; 
  for(auto pos: position_vec){
    object_position.push_back(position2cvPoint3f(pos));
  }
  for(int i=0; i<boards_thermal[0].idx.size(); ++i){
    for(int j=0; j<boards_thermal[0].idx[i].size(); ++j){
      if(boards_thermal[0].idx[i][j]<0) continue;
      cv::Point2f pixel;
      pixel.x = corners_thermal.p[boards_thermal[0].idx[i][j]].x;
      pixel.y = corners_thermal.p[boards_thermal[0].idx[i][j]].y;
      image_pixels.push_back(pixel);
    }
  }
  obj.push_back(object_position); img.push_back(image_pixels);
  cv::Mat rvec, tvec; // Extrinsic placeholder
  cv::calibrateCamera(obj, img, thermal.size(), thermal_intrinsic, thermal_distortion, rvec, tvec, CV_CALIB_USE_INTRINSIC_GUESS);
  cv::Mat thermal_to_rgb = cv::Mat::zeros(rgb.size(), thermal.type());
  tf::Transform homo = RT2tf(rvec, tvec);
  write_file(thermal_intrinsic, thermal_distortion, homo); 
  std::cout << "Extrinsic matrix write to: camera_model.txt\n";
  for(int i=0; i<rgb.cols; ++i){
    for(int j=0; j<rgb.rows; ++j){
      tf::Vector3 rgb_xyz = position2tf(getPosition(i, j, intrinsic, depth, true, kernel_size));
      if(rgb_xyz.getZ()==0.0) continue; // Ignore if z equals to zero
      tf::Vector3 thermal_xyz = homo*rgb_xyz;
      int u, v;
      project2pixel_plane(thermal_xyz, thermal_intrinsic, thermal_distortion, u, v);
      if(u>=0 && u<thermal.cols && v>=0 && v<thermal.rows){
        thermal_to_rgb.at<unsigned char>(j, i) = thermal.at<unsigned char>(v, u);
      }
    }
  }
  cv::imwrite("thermal_to_rgb.jpg", thermal_to_rgb);
  cv::Mat color_map; cv::applyColorMap(thermal_to_rgb, color_map, cv::COLORMAP_JET);
  cv::Mat combined;
  cv::addWeighted(rgb, 1.0, color_map, 0.3, 0.0, combined);
  cv::imwrite("combined.jpg", combined);
  return 0;
}

#include <chrono>
#include <cali/align_thermal_to_rgb.h>

int main(int argc, char** argv){
  if(argc!=6){
    std::cout << "\033[1;33mNot enough input arguments, exiting\033[0m\n";
    std::cout << "\033[1;33m./example [camera_model_path] [rgb_intrinsic_path] [rgb_image_path] [depth_image_path] [thermal_image_path]\033[0m\n";
    exit(EXIT_FAILURE);
  }
  align_thermal_to_rgb foo(argv[1]);
  Intrinsic intrinsic;
  parse_rgb_intrinsic(argv[2], intrinsic);
  foo.set_rgb_intrinsic(intrinsic);
  cv::Mat dst;
  cv::Mat rgb = cv::imread(argv[3], CV_LOAD_IMAGE_COLOR);
  cv::Mat depth = cv::imread(argv[4], CV_LOAD_IMAGE_ANYDEPTH);
  cv::Mat thermal = cv::imread(argv[5], CV_LOAD_IMAGE_GRAYSCALE);
  if(rgb.empty() || depth.empty() || thermal.empty()){
    std::cout << "\033[1;33mNo image read, exiting\033[0m\n";
    exit(EXIT_FAILURE);
  }
  foo.set_rgb_image(rgb);
  foo.set_depth_image(depth);
  foo.set_thermal_image(thermal);
  auto s_ts = std::chrono::high_resolution_clock::now();
  foo.align(dst);
  auto e_ts = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(e_ts-s_ts).count();
  std::cout << "Alignemnt spend " << duration*1e-6 << " ms\n";
  cv::imwrite("aligned.jpg", dst);
  foo.show_model();
  return 0;
}

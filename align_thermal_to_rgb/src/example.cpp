#include <cali/align_thermal_to_rgb.h>

int main(int argc, char** argv){
  align_thermal_to_rgb foo(argv[1]);
  Intrinsic intrinsic{612.71576, 612.72723, 323.80862, 238.39876};
  foo.set_rgb_intrinsic(intrinsic);
  cv::Mat dst;
  cv::Mat rgb = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
  cv::Mat depth = cv::imread(argv[3], CV_LOAD_IMAGE_ANYDEPTH);
  cv::Mat thermal = cv::imread(argv[4], CV_LOAD_IMAGE_GRAYSCALE);
  foo.set_rgb_image(rgb);
  foo.set_depth_image(depth);
  foo.set_thermal_image(thermal);
  foo.align(dst);
  cv::imwrite("aligned.jpg", dst);
  foo.show_model();
  return 0;
}

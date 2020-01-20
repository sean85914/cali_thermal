#include <chrono>
#include <cali/align_thermal_to_rgb.h>

int main(int argc, char** argv){
  if(argc!=7){
    std::cout << "\033[1;33mNot enough input arguments, exiting\033[0m\n";
    std::cout << "\033[1;33m./example [camera_model_path] [rgb_intrinsic_path] [rgb_image_path] [depth_image_path] [thermal_image_path] [outut_file_name]\033[0m\n";
    exit(EXIT_FAILURE);
  }
  align_thermal_to_rgb foo(argv[1]); // Result from `get_extrinsic`
  Intrinsic intrinsic;
  parse_rgb_intrinsic(argv[2], intrinsic); // Note that there are three methods for setting RGB intrinsic
  foo.set_rgb_intrinsic(intrinsic);
  cv::Mat dst;
  cv::Mat rgb = cv::imread(argv[3], CV_LOAD_IMAGE_COLOR);
  cv::Mat depth = cv::imread(argv[4], CV_LOAD_IMAGE_ANYDEPTH);
  cv::Mat thermal = cv::imread(argv[5], CV_LOAD_IMAGE_GRAYSCALE);
  if(rgb.empty() || depth.empty() || thermal.empty()){
    std::cout << "\033[1;33mNo image read, exiting\033[0m\n";
    exit(EXIT_FAILURE);
  }
  // Set three images
  foo.set_rgb_image(rgb);
  foo.set_depth_image(depth);
  foo.set_thermal_image(thermal);
  auto s_ts = std::chrono::high_resolution_clock::now();
  foo.align(dst);
  auto e_ts = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(e_ts-s_ts).count();
  // Print conversion time
  std::cout << "Alignemnt spend " << duration*1e-6 << " ms\n";
  std::string output_file(argv[6]);
  if(output_file.find(".jpg")==std::string::npos){
    cv::imwrite(output_file+".jpg", dst);
  }else
    cv::imwrite(output_file, dst);
  // Print model parameters
  foo.show_model();
  cv::Mat color_map; cv::applyColorMap(dst, color_map, cv::COLORMAP_JET);
  cv::Mat combined;
  cv::addWeighted(rgb, 1.0, color_map, 0.3, 0.0, combined);
  std::string substr;
  std::size_t extension_pos = output_file.find(".jpg");
  substr = output_file.substr(0, extension_pos);
  substr += "_combined.jpg";
  cv::imwrite(substr, combined);
  return 0;
}

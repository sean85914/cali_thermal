## DEPENDENCIES
1. CMake >= 2.8.3
2. OpenCV >= 3.0
3. ROS/tf

## STRUCTURE ORGANIZATION
- libcbdetect: C++ implmentation for chessboard detection from [this repo](https://github.com/ftdlyc/libcbdetect)
- align_thermal_to_rgb: align thermal image to RGB image (in this case, Intel RealSense D435) with the help of chessboard
  - include
    - cali
      - align_thermal_to_rgb.h: class definition for `align_thermal_to_rgb`
  - src
    - helper.hpp: some helper functions
    - align_thermal_to_rgb.cpp: class implementation for `align_thermal_to_rgb`
    - get_extrinsic.cpp: intrinsic/ extrinsic calibration for thermal camera, user provideds **rgb, depth and thermal images**, and **an interger for mean filter kernel size**, then it will generate a file with **thermal camera intrinsic, distortion coefficients** and **rgb-to-thermal extrinsic**, named in `camera_model.txt`
    - example.cpp: `align_thermal_to_rgb` usage example
  - bin
    - get_extrinsic -> src/get_extrinsic.cpp
    - example -> src/example.cpp
  - lib
    - libalign_thermal_to_rgb.so: library for `align_thermal_to_rgb`, link this library in yourown source code
  - result: some concrete alignment result
  - CMakeLists.txt
 
## BUILD
* Since the `align_thermal_to_rgb` should do chessboard detection, so we have to build the `libcbdetect` first
  1. cd && git clone https://github.com/sean85914/align_thermal_to_rgb.git
  2. cd align_thermal_to_rgb/cali/libcbdetect
  3. mkdir devel && cd devel && cmake ../ && make
* Then we can build the `align_thermal_to_rgb`
  4. cd ~/align_thermal_to_rgb/cali/align_thermal_to_rgb 
  5. mkdir devel && cd devel && cmake ../ && make && cd ../bin
  6. Then, if you have data for testing, run 
  `./get_extrinsic [path_to_rgb_image] [path_to_depth_image] [path_to_thermal_image] 7`
    Then you will get the output file **camera_model.txt**, an aligned thermal image and a combined one
  7. With this file, you can use `align_thermal_to_rgb` class, while remember to set the rgb camera intrinsic with

## RESULT
![image](github.com/sean85914/align_thermal_to_rgb/blob/master/cali/align_thermal_to_rgb/result/result.jpg)


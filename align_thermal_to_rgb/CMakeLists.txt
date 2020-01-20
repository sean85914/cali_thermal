cmake_minimum_required(VERSION 2.8.3)
add_compile_options(--std=c++11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
find_package(OpenCV REQUIRED)
find_package(tf REQUIRED)
find_package(Eigen3 REQUIRED)

string(REPLACE "align_thermal_to_rgb" "libcbdetect" libcbdetection ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${libcbdetection}/include
  ${tf_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

message(STATUS ${tf_INCLUDE_DIRS})
string(REPLACE "align_thermal_to_rgb" "libcbdetect/lib/libcbdetect.so" libcbdetection_lib ${CMAKE_CURRENT_SOURCE_DIR})

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

if(CMAKE_BUILD_TYPE STREQUAL Debug)
  ADD_DEFINITIONS(-DDEBUG)
ENDIF()

add_library(align_thermal_to_rgb src/align_thermal_to_rgb.cpp)
target_link_libraries(align_thermal_to_rgb ${OpenCV_LIBRARIES} ${tf_LIBRARIES})

add_executable(get_extrinsic src/get_extrinsic.cpp)
target_link_libraries(get_extrinsic ${OpenCV_LIBRARIES} ${libcbdetection_lib} ${tf_LIBRARIES})

add_executable(example src/example.cpp)
target_link_libraries(example ${OpenCV_LIBRARIES} ${libcbdetection_lib} ${tf_LIBRARIES} align_thermal_to_rgb)

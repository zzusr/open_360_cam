cmake_minimum_required(VERSION 2.8.3)
project(fish360)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system)
# catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  actionlib_msgs
  actionlib
)

find_package(catkin REQUIRED COMPONENTS 
rostime 
sensor_msgs 
message_filters 

)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES sapr
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(fish360 src/fish360.cpp)

execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)

add_dependencies(fish360 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fish360
  ${OpenCV_LIBRARIES}

  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libholiday.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libSeetaFaceCropper200.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libSeetaFaceDetector200.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libSeetaFaceRecognizer200.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libSeetaPointDetector200.so -ldl -pthread

  ${catkin_LIBRARIES} -lcurl -ljsoncpp
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libmsc.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libasound.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libasound.so.2 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libasound.so.2.0.0 -ldl -pthread

  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_calib3d.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_core.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_dnn.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_features2d.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_flann.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_highgui.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_imgcodecs.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_imgproc.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_ml.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_objdetect.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_photo.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_shape.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_stitching.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_superres.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_video.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_videoio.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_videostab.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_viz.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_calib3d.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_core.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_dnn.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_features2d.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_flann.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_highgui.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_imgcodecs.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_imgproc.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_ml.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_objdetect.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_photo.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_shape.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_stitching.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_superres.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_video.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_videoio.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_videostab.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_viz.so.3.3 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_calib3d.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_core.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_dnn.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_features2d.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_flann.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_highgui.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_imgcodecs.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_imgproc.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_ml.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_objdetect.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_photo.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_shape.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_stitching.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_superres.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_video.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_videoio.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_videostab.so.3.3.1 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libopencv_viz.so.3.3.1 -ldl -pthread


)

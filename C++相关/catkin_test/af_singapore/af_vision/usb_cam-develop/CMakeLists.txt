cmake_minimum_required(VERSION 2.8.3)
project(usb_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS image_transport image_view roscpp std_msgs std_srvs sensor_msgs camera_info_manager cv_bridge)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

find_package(OpenCV)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)
###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp)
target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)
## save image executable

add_executable(save_image nodes/save_image.cpp)
target_link_libraries(save_image
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(save_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(save_image ${CMAKE_THREAD_LIBS_INIT})


add_executable(BiaoDingSave src/BiaoDingSave.cpp)
target_link_libraries(BiaoDingSave ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(BiaoDingSave2 src/BiaoDingSave2.cpp)
target_link_libraries(BiaoDingSave2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



install(TARGETS save_image ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

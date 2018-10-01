cmake_minimum_required(VERSION 2.8.3)
project(wam_orbslam_solver)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge std_msgs image_transport message_generation)
#find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(VISP REQUIRED)



add_message_files(
    FILES
        pathData.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS
    message_runtime
    #pcl
    #pcl_ros
)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES signature_drawing
#)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

include_directories ("${PROJECT_SOURCE_DIR}/include")
add_subdirectory ("${PROJECT_SOURCE_DIR}/lib")

include_directories(
include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
#include_directories(${VISP_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(solver src/solver.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(solver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #${PCL_LIBRARIES}
)
#target_link_libraries(signature_vis ${VISP_LIBRARIES})

#target_link_libraries(signature_vis lapack blas)
target_link_libraries (solver lib )#${PCL_LIBRARIES}

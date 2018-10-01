find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge std_msgs image_transport message_generation)
#find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        #pcl
        #pcl_ros
)

include_directories(
include

${catkin_INCLUDE_DIRS}

${OpenCV_INCLUDE_DIRS}
)

#include_directories(${PCL_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS})

#add_definitions(${PCL_DEFINITIONS})

add_library (
lib
math_helper.cpp
string_convertor.cpp
transformation2D.cpp
keyFrame.cpp
colormod.cpp
chris_ICP.cpp
pair_solver.cpp

#Lib ICPs, copyright to Andreas Geiger geiger@kit.edu
#icp.cpp
#icpPointToPlane.cpp
#icpPointToPoint.cpp
#kdtree.cpp
#matrix.cpp
)

#target_link_libraries (lib ${PCL_LIBRARIES})

find_package(OpenCV REQUIRED)
include_directories(
include
${OpenCV_INCLUDE_DIRS}
)
add_library (lib math_helper.cpp string_convertor.cpp transformation2D.cpp)

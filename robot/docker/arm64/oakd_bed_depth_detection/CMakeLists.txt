# SPECIFY THE MINIMUM VERSION OF CMAKE REQUIRED
cmake_minimum_required(VERSION 2.8.12)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)

# SPECIFY YOUR PROJECT NAME
PROJECT(OpenCV_CPP)

# ENABLE C++11
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# MAKE SURE OPENCV IS INSTALLED CORRECTLY
find_package( OpenCV REQUIRED )
# find_package(depthai CONFIG REQUIRED)

# INCLUDE OPENCV DIRECTORIES
include_directories( ${OpenCV_INCLUDE_DIRS})

# MACRO TO COMPILE CPP FILES
# Do Not Edit
MACRO(add_example name)
  ADD_EXECUTABLE("${name}" ${name}.cpp)
#   TARGET_LINK_LIBRARIES(${name} ${OpenCV_LIBS} )
  TARGET_LINK_LIBRARIES(${name} PUBLIC depthai::opencv ${OpenCV_LIBS} )
  if(NOT MSVC)
    target_compile_options(${name} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Werror=return-type>)
    endif()
ENDMACRO()

# COMPILE CPP FILES USING THIS LINE
######## EDIT THE FILE NAMES ########
add_example(main)
add_example(bed_detection)
add_example(yolov4-spatial)
add_example(spatial_yolov4_bed)
add_example(rgb_tiny_yolo)
add_example(mobilenet_ssd)

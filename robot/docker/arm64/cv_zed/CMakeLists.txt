cmake_minimum_required(VERSION 3.5.1)
project(cv_zed)

# set the C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

find_package(OpenCV 4 REQUIRED)
find_library(ZBAR_LIBRARIES NAMES zbar)
find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

set(_include_dirs "include;/usr/include;/usr/include/opencv4")

add_executable(qr_code src/qr_code.cpp)
target_link_libraries(qr_code zbar ${OpenCV_LIBRARIES})

add_executable(qr_code_video_wc src/qr_code_video_wc.cpp)
target_link_libraries(qr_code_video_wc zbar ${OpenCV_LIBRARIES})

add_executable(qr_code_video_zed2 src/qr_code_video_zed2.cpp)
target_link_libraries(qr_code_video_zed2 zbar ${OpenCV_LIBRARIES} ${ZED_LIBS})


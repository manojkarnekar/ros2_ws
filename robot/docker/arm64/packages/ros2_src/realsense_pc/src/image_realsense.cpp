#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout

// #include "example.hpp"          // Include short list of convenience functions for rendering

// This example will require several standard data-structures and algorithms:
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/cvconfig.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
using pixel = std::pair<int, int>;

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data


int main(int argc, char * argv[]) try
{
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);

    rs2::frameset frames = p.wait_for_frames();

    auto ir_frame_left = frames.get_infrared_frame(1);
    auto ir_frame_right = frames.get_infrared_frame(2);
    auto depth = frames.get_depth_frame();
    auto colored_frame = frames.get_color_frame();


    //save files
    cv::Mat dMat_left = cv::Mat(cv::Size(1280, 720), CV_8UC1, (void*)ir_frame_left.get_data());
    cv::Mat dMat_right = cv::Mat(cv::Size(1280, 720), CV_8UC1, (void*)ir_frame_right.get_data());
    cv::Mat dMat_colored = cv::Mat(cv::Size(1920, 1080), CV_8UC3, (void*)colored_frame.get_data());


    cv::imwrite( "/home/manoj/sar/docker/arm64/packages/ros2_src/realsense_pc/irLeft.png", dMat_left );
    cv::imwrite( "/home/manoj/sar/docker/arm64/packages/ros2_src/realsense_pc/irRight0.png", dMat_right );
    cv::imwrite( "/home/manoj/sar/docker/arm64/packages/ros2_src/realsense_pc/coloredCV0.png", dMat_colored);

    while (true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        auto width = depth.get_width();
        auto height = depth.get_height();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);

        // Print the distance
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \r" << endl;
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
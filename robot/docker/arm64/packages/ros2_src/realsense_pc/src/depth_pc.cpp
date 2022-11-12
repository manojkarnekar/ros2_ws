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

using namespace std;
using pixel = std::pair<int, int>;

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data

vector<float> pixel_to_point(const rs2::depth_frame& frame, pixel u)
{
    float upixel[2];
    float upoint[3];
    upixel[0] = static_cast<float>(u.first);
    upixel[1] = static_cast<float>(u.second);  
    auto udist = frame.get_distance(static_cast<int>(upixel[0]), static_cast<int>(upixel[1]));

    // Deproject from pixel to point in 3D
    rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);

    return {upoint[0], upoint[1], upoint[2]} ;
 
}


int main(int argc, char * argv[]) try
{
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    while (true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        auto width = depth.get_width();
        auto height = depth.get_height();

        pixel u = {width / 4, height / 2};
        float x ,y, z;
        x = pixel_to_point(depth , u)[0];
        y = pixel_to_point(depth , u)[1];
        z = pixel_to_point(depth , u)[2];
        cout << "x = " << x << " y = " << y <<" z = " << z <<endl;
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
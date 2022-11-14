#include <sl/Camera.hpp>
#include <vector>
#include<iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <chrono>
#include <memory>
#include <math.h>

using namespace std;
using namespace sl;

// vector<int> image_to _xy(sl::Mat image)
// {
//     int x = image.getWidth() / 2;
//     int y = image.getHeight() / 2;
//     vector<int> pixel {};
//     pixel.push_back(x);
//     pixel.push_back(y);
//     return pixel;
// }
sl::float4 pixel_to_point_cloud(sl::Mat image)
{
    sl::float4 point_cloud_value;
    int x = image.getWidth() / 2;
    int y = image.getHeight() / 2;
    point_cloud.getValue(x, y, &point_cloud_value);
    return point_cloud_value;
}

float Sqrt(sl::float4 point_cloud_value)
{
    if(std::isfinite(point_cloud_value.z))
    {
        float distance = hypot(point_cloud_value.x, 
                        point_cloud_value.y, 
                        point_cloud_value.z);
        return distance;
    }
    else
    {
        return 0;
    }
}


float min_val(vector<float> v)
{
  auto it = std::minmax_element(v.begin(), v.end());
  float min = *it.first;
  return min;
}




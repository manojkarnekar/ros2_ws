#ifndef AHRS_HPP
#define AHRS_HPP
#include <iostream>
#include <vector>
#include <bits/stdc++.h>

using namespace std;

class Lidar
{
public:
    //  header_;
    std::string frame_id_;
    float angle_min_;
    float angle_max_;
    float angle_increment_;
    float time_increment_;
    float scan_time_;
    float range_min_;
    float range_max_;
    vector<float> intensities_{};
    vector<float> ranges_{};

    vector<float> intensities_pop{};
    vector<float> ranges_pop{};
    // float intensities_;
    // float ranges_;
};

#endif
#ifndef mic_vision_H
#define mic_vision_H

#include <memory>
#include <chrono>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <math.h>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdlib>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>



#include <nav_msgs/srv/get_map.hpp>
#include <std_srvs/srv/trigger.hpp>


using namespace std;

namespace micvision
{
    using Point = Eigen::Vector2f;
    using Pixel = Eigen::Vector2i;

    class GridMap 
    {
        private:
            nav_msgs::msg::OccupancyGrid occupancy_grid_;
            unsigned int map_width_;
            unsigned int map_height_;
            signed char lethal_cost_;

        public:
            void update(nav_msgs::msg::OccupancyGrid grid) 
            {
                occupancy_grid_ = grid;
                map_width_ = occupancy_grid_.info.width;
                map_height_ = occupancy_grid_.info.height;
                cout<<"width = "<<map_width_<<" hieght = "<<map_height_<<endl;
                cout<<"Got new map of size"<<endl; 
            }

            unsigned int getWidth() const {return map_width_;}
            unsigned int getHeight() const {return map_height_;}
            unsigned int getSize() const {return map_width_ * map_height_;}
            double getResolution() const {return occupancy_grid_.info.resolution;}
            double getOriginX() const {return occupancy_grid_.info.origin.position.x;}
            double getOriginY() const {return occupancy_grid_.info.origin.position.y;}

            signed char getLethalCost() const {return lethal_cost_;}
            void setLethalCost(signed char c) {lethal_cost_ = c;}

            const nav_msgs::msg::OccupancyGrid& getMap() const {return occupancy_grid_;}

            // Get the array index from the given x,y coordinates
            bool getIndex(const unsigned int x, 
            const unsigned int y, 
            unsigned int& i) const 
            {
                if (x >= map_width_ || y >= map_height_) 
                {
                    return false;
                }
                i = y * map_width_ + x;
                return true;
            }   

            bool getIndex(const Pixel& pixel, 
            unsigned int& i) const 
            {
                const auto x = pixel(0);
                const auto y = pixel(1);
                return getIndex(x, y, i);
            }

            // Get the x,y coordinates from the given array index
            bool getCoordinates(unsigned int& x,
                      unsigned int& y,
                      const unsigned int i) const 
            {
                if (i >= map_width_ * map_height_) 
                {
                    cout<<"getCoordinates() failed!";
                    return false;
                }
                y = i / map_width_;
                x = i % map_width_;
                return true;
            }

            bool getCoordinates(Pixel& pixel, 
            const unsigned int i) const 
            {
                if (i >= map_width_ * map_height_) 
                {
                    cout<<"getCoordinates() failed!";
                    return false;
                }
                    pixel = Pixel(i % map_width_, i / map_width_);
                    return true;
            }

            // Index based methods
            signed char getData(unsigned int index) const 
            {
                if (index < map_width_ * map_height_)
                {
                    return occupancy_grid_.data[index];
                }
                else
                {
                    return -1;
                }  
            }

            bool setData(unsigned int index, signed char value) 
            {
                if (index >= map_width_ * map_height_) 
                {
                    return false;
                }
                occupancy_grid_.data[index] = value;
                return true;
            }

            bool isFree(unsigned int index) const 
            {
                signed char value = getData(index);
                if (value >= 0 && value < lethal_cost_) return true;
                return false;
            }

            bool isFrontier(unsigned int index) const 
            {
                int y = index / map_width_;
                int x = index % map_width_;

                if (getData(x - 1, y - 1) == -1) return true;
                if (getData(x - 1, y) == -1) return true;
                if (getData(x - 1, y + 1) == -1) return true;
                if (getData(x, y - 1) == -1) return true;
                if (getData(x, y + 1) == -1) return true;
                if (getData(x + 1, y - 1) == -1) return true;
                if (getData(x + 1, y) == -1) return true;
                if (getData(x + 1, y + 1) == -1) return true;

                return false;
            }

            /** Gets indices of all free neighboring cells with given offset */
            std::vector<unsigned int> getFreeNeighbors(unsigned int index,
                                             int offset = 1) const 
            {
                std::vector<unsigned int> neighbors;

                if (offset < 0) offset *= -1;
                int y = index / map_width_;
                int x = index % map_width_;

                for (int i = -offset; i <= offset; i++)
                for (int j = -offset; j <= offset; j++)
                    if (getIndex(x + i, y + j, index) && isFree(index))
                    neighbors.push_back(index);

                return neighbors;
            }

            /** Gets indices of all neighboring cells */
            std::vector<unsigned int> getNeighbors(unsigned int index,
                                         bool diagonal = false) const 
            {
                std::vector<unsigned int> neighbors;

                int y = index / map_width_;
                int x = index % map_width_;
                unsigned int i;
                if (getIndex(x - 1, y,   i)) neighbors.push_back(i);   
                if (getIndex(x + 1, y,   i)) neighbors.push_back(i);
                if (getIndex(x,   y - 1, i)) neighbors.push_back(i);
                if (getIndex(x,   y + 1, i)) neighbors.push_back(i);

                if (diagonal) {
                if (getIndex(x - 1, y - 1, i)) neighbors.push_back(i);
                if (getIndex(x - 1, y + 1, i)) neighbors.push_back(i);
                if (getIndex(x + 1, y - 1, i)) neighbors.push_back(i);
                if (getIndex(x + 1, y + 1, i)) neighbors.push_back(i);
                }
                return neighbors;
            }

            // Coordinate based methods
            signed char getData(int x, int y) const 
            {
                if (x < 0 || x >= (int)map_width_ || y < 0 || y >= (int)map_height_)
                return 100;
                else
                return occupancy_grid_.data[y * map_width_ + x];
            }

            signed char getData(const Pixel& pixel) const 
            {
                return getData(pixel(0), pixel(1));
            }

            signed char getRawData(int x, int y) const 
            {
                return occupancy_grid_.data[y * map_width_ + x];
            }

            signed char getRawData(const Pixel& pixel) const 
            {
                return getRawData(pixel(0), pixel(1));
            }

            bool setData(int x, int y, signed char value) 
            {
                if (x < 0 || x >= (int)map_width_ || y < 0 || y >= (int)map_height_) {
                return false;
                }
                occupancy_grid_.data[y * map_width_ + x] = value;
                return true;
            }

            bool setData(const Pixel& pixel, signed char value) 
            {
                return setData(pixel(0), pixel(1), value);
            }

            bool isFree(int x, int y) const 
            {
                signed char value = getData(x, y);
                if (value >= 0 && value < lethal_cost_) return true;
                return false;
            }

            bool isFree(const Pixel& pixel) const 
            {
                return isFree(pixel(0), pixel(1));
            }

            float getBoundaryX() const 
            {
                return getOriginX() + getWidth() * getResolution();
            }

            float getBoundaryY() const 
            {
                return getOriginY() + getHeight() * getResolution();
            }

};

}



#endif
//#include "follow_path1.cpp"
//#include "localization_util.cpp"
#include "mic_vision/mic_vision.h"
//#include <micvision/localization.h>
// #include <micvision/grid_map.h>
// #include <micvision/commands.h>

using namespace std;
using namespace std::chrono_literals;
namespace micvision {
  typedef std::vector<Eigen::Vector3f> PointCloud;
  typedef std::vector<micvision::Pixel> PointCloudUV;
}//;
                //typedef 
            //typedef
using std::placeholders::_1;
using std::placeholders::_2;
// nav_msgs::srv::GetMap::Request::SharedPtr request_message;
// nav_msgs::srv::GetMap::Response::SharedPtr response_message;
// int uv;      //const 
// int index;       //const 
micvision::GridMap current_map_;
rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client;
Eigen::Vector2i best_position_;

  struct LaserScanSample {
  micvision::PointCloudUV point_cloud;
  std::vector<int> indices;
  int min_x, max_x;
  int min_y, max_y;
  double robot_radius_ = 0.2;
};

  struct CellData {
  CellData() = delete;
  CellData(double d, unsigned int i, unsigned int x, unsigned int y):
    distance(d), index(i), x(x), y(y) {}
  double distance;
  unsigned int index, x, y;

  friend bool operator<(const CellData& c1, const CellData& c2) {
    return c1.distance > c2.distance;
  }
};

class MicvisionLocalization : public rclcpp::Node                                    //    //public path_fol 
{
  public:
    explicit MicvisionLocalization() : Node("MicvisionLocalization")
    {

      subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 5, std::bind(&MicvisionLocalization::mapCallback, this, _1));
      subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 1, std::bind(&MicvisionLocalization::scanCallback, this, _1));
      subscription2_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&MicvisionLocalization::odomCallback, this, _1));
      // subscription3_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      // "/amcl_pose", 1, std::bind(&MicvisionLocalization::debugAPosition, this, _1));

      // timer1_ = this->create_wall_timer(
      // 1s, std::bind(&MicvisionLocalization::tracking, this));

      publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

      timer2_ = this->create_wall_timer(
      500ms, std::bind(&MicvisionLocalization::init_pose_callback, this));

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // server_ = this->create_service<nav_msgs::srv::GetMap>("/static_map", std::bind(&MicvisionLocalization::getMap1, this, _1, _2)); //, _1

      get_map_client_ =
      this->create_client<nav_msgs::srv::GetMap>("static_map");

    }

  public:
    // rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr server_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;
    // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription3_;

    rclcpp::Time now = this->get_clock()->now();

    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_client_;

    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr get_map_client_{nullptr};

    //   localization_server_ =
    // nh.advertiseService(LOCATION_SERVICE,
    //                     &MicvisionLocalization::receiveLocalization, this);

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;

    struct LaserScanSample transformPointCloud(const Eigen::Quaternionf& transform);
    void add(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
          std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void odomCallback(const nav_msgs::msg::Odometry& odom);
    void scanCallback(const sensor_msgs::msg::LaserScan& scan);
    void debugAPosition(geometry_msgs::msg::PoseWithCovarianceStamped& pose);
    void handleLaserScan();
    void getMap();
    // void getMap1(nav_msgs::srv::GetMap::Request::SharedPtr request_message, nav_msgs::srv::GetMap::Response::SharedPtr response_message);

    bool validPosition(const int uv, const int index);
    double scoreASample(const LaserScanSample& sample,
                                           const int u, const int v);
    void init_pose_callback(); 
    bool receiveLocalization(std_srvs::srv::Trigger_Request& req, std_srvs::srv::Trigger_Response& res);
    void computeCaches();
    void inflateMap();
    void shutdown();
    void enqueueObstacle(const unsigned int index, const unsigned int x, const unsigned int y);
  inline double distanceLookup(const unsigned int mx,
                               const unsigned int my,
                               const unsigned int sx,
                               const unsigned int sy) const;    
  inline signed char costLookup(const unsigned int mx,
                                const unsigned int my,
                                const unsigned int sx,
                                const unsigned int sy) const;
    // void tracking();
    std::vector<micvision::Pixel> bresenham(const micvision::Pixel& start, const micvision::Pixel& end);
    void delete_params();

    std::string map_frame_ ="map";
    std::string robot_frame_ ="base_link";

    int width_, height_;
    double resolution_;
    micvision::GridMap current_map_;
    unsigned int cell_inflation_radius_;
    signed char** cached_costs_ = nullptr;
    double** cached_distances_ = nullptr;
    double inflation_radius_ = 0.3;
    unsigned char* inflation_markers_ = nullptr;
    std::priority_queue<CellData> inflation_queue_;
    std::vector<std::pair<bool, double> > inflated_map_data_;
    signed char cost_obstacle_ = 100;
    double current_position_score_ = 0.0f;
    bool big_angle_twist_ = false;
    double best_angle_;  

    Eigen::Vector2i best_position_;
    int laserscan_anglar_step_ = 6;
    double robot_radius_ = 0.2;
    int laserscan_circle_step_ = 6;
    int range_step_ = 3;
    double min_valid_range_ = 0.0f;
    double max_valid_range_ = 10.0f;
    bool quick_score_ = true;
    int quick_score_num_ = 8;

  std::vector<LaserScanSample> laserscan_samples_;
  micvision::PointCloud point_cloud_;
  bool handling_lasescan_ = false;
  bool has_new_map_ = true;
  double PI_2 = 2 * M_PI;      //constexpr
  double RADIAN_PRE_DEGREE = M_PI / 180;      //constexpr 

};

std::vector<micvision::Pixel> MicvisionLocalization::bresenham(const micvision::Pixel& start,
                                                    const micvision::Pixel& end) {
                                                  
  int x = start(0), y = start(1);
  int dx = abs(end(0) - x);
  int dy = abs(end(1) - y);

  const int ux = end(0) > x ? 1 : -1;
  const int uy = end(1) > y ? 1 : -1;

  bool exchange = false;      // exchange dx and dy?
  if (dy > dx) {
    std::swap(dx, dy);
    exchange = true;
  }

  int p = 2 * dy - dx;
  std::vector<micvision::Pixel> result;

  for (int i = 0; i <= dx; ++i) {
    // RCLCPP_INFO(get_logger(),"x: %d, y: %d", x, y);
    if (x <= 5 || x >= current_map_.getWidth() - 5 ||
        y <= 5 || y >= current_map_.getHeight() - 5)
      break;
    result.push_back(micvision::Pixel(x, y));
    if (p >= 0) {
      if (!exchange)
        y += uy;
      else
        x += ux;
      p -= 2 * dx;
    }

    if (!exchange)
      x += ux;
    else
      y += uy;
    p += 2 * dy;
  }
  return result;
}

inline micvision::Pixel floor(const Eigen::Vector3f& v) {
  return micvision::Pixel(std::lround(v[0] - 0.5), std::lround(v[1] - 0.5));
}

struct LaserScanSample MicvisionLocalization::transformPointCloud(
  const Eigen::Quaternionf& transform) {
  micvision::PointCloudUV result;
  result.reserve(point_cloud_.size());
  std::vector<int> indices;
  indices.reserve(point_cloud_.size());
  const float resolution = current_map_.getResolution();
  int min_x = width_, max_x = -min_x;
  int min_y = height_, max_y = -min_y;
  for (const Eigen::Vector3f& point : point_cloud_) {
    // result.emplace_back(transform * point);
    const micvision::Pixel temp = floor(transform * point / resolution);
    min_x = min_x < temp[0] ? min_x : temp[0];
    min_y = min_y < temp[1] ? min_y : temp[1];
    max_x = max_x > temp[0] ? max_x : temp[0];
    max_y = max_y > temp[1] ? max_y : temp[1];
    result.emplace_back(temp);
    indices.emplace_back(temp[0] + temp[1] * width_);
  }
  return LaserScanSample{result, indices, min_x, max_x, min_y, max_y};
}

void MicvisionLocalization::delete_params() {
  delete[] inflation_markers_;
  delete[] cached_distances_;
  delete[] cached_costs_;
}

void MicvisionLocalization::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  RCLCPP_INFO(get_logger(),"mapCallback");
  // only inflate the map when the first map or a new map comes
  // if (!getMap(request_message, response_message)) {
  //   RCLCPP_INFO(get_logger(),"Could not get a new map, trying to go with the old one...");
  //   return;
  // }
  //getMap();
  inflateMap();
}


void MicvisionLocalization::odomCallback(const nav_msgs::msg::Odometry& odom) {
  big_angle_twist_ =
    (std::abs(odom.twist.twist.angular.z) >= 0.1) ? true : false;
  cout<<"big_angle_twist_ = "<<big_angle_twist_<<endl;

}

void MicvisionLocalization::scanCallback(const sensor_msgs::msg::LaserScan& scan) {
  RCLCPP_INFO(get_logger(),"scanCallback, scan size:");   //%d", scan.ranges.size()
  cout<<"This is laser scan"<<endl;

  //mtx.lock();
  if (rclcpp::ok()) {
    point_cloud_.clear();
    // generate the point_cloud_
    float angle = scan.angle_min;
    for (int i = 0; i < scan.ranges.size(); i += laserscan_circle_step_) {
      const float range = scan.ranges[i];
      if (min_valid_range_ <= range && range <= max_valid_range_) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        point_cloud_.push_back(rotation * (range * Eigen::Vector3f::UnitX()));
      }

      angle += scan.angle_increment * laserscan_circle_step_;
    }
    // handleLaserScan();
  }
  //mtx.unlock();
}

void MicvisionLocalization::handleLaserScan() {
  //mtx.lock();
  laserscan_samples_.clear();
  double angle = -M_PI;
  laserscan_samples_.reserve(static_cast<int>(
                               PI_2 / RADIAN_PRE_DEGREE / laserscan_anglar_step_));
  while (angle <= M_PI) {
    // angle = -M_PI + laserscan_anglar_step_ * N
    laserscan_samples_.emplace_back(transformPointCloud(
                                      Eigen::Quaternionf(
                                        Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()))));
    angle += laserscan_anglar_step_ * RADIAN_PRE_DEGREE;
  }
  //mtx.unlock();
}


bool MicvisionLocalization::validPosition(const int uv, const int index) {     //const const 
  micvision::Pixel start(uv % width_, uv / width_),
        end((uv + index) % width_, (uv + index) / width_);
  /*
   *RCLCPP_INFO(get_logger(),"start: %d, %d", start(0), start(1));
   *RCLCPP_INFO(get_logger(),"end: %d, %d", end(0), end(1));
   */

  const auto line = bresenham(start, end);

  int i;
  for (i = 0; i < line.size(); ++i) {
    const auto l = line[i](0) + line[i](1) * width_;
    if (inflated_map_data_[l].second > 0.5)
      break;
  }

  if (line.size() - i > 5)
    return false;
  else
    return true;
}

double MicvisionLocalization::scoreASample(const LaserScanSample& sample,
                                           const int u, const int v) {
  double score = 0.0;
  if (quick_score_) {
    int step = sample.point_cloud.size() / quick_score_num_;
    int object = 0;
    for (int i = 0; i < sample.point_cloud.size(); i += step) {
      if (current_map_.getRawData(sample.point_cloud[i][0] + v,
                                  sample.point_cloud[i][1] + u) >= 50)
        object++;
    }
    if (object <= quick_score_num_ / 2) return 0;
  }
  for (auto s : sample.point_cloud) {
    // s = [width, height, z]
    score += current_map_.getRawData(s[0] + v, s[1] + u);
  }
  // RCLCPP_INFO(get_logger(),"score: %f", score);
  return score;
}

void MicvisionLocalization::init_pose_callback() {
  // first we need handle the point_cloud_
  handleLaserScan();
  RCLCPP_INFO(get_logger(),"start score");
  // RCLCPP_INFO(get_logger(),"score the laserscan samples.");
  double score = 0.5;
  int count = 0;

  // ROS_INFO_STREAM("inflated map data size: " << inflated_map_data_.size());
  const int sample_size = laserscan_samples_[0].point_cloud.size();
  const int step = sample_size / quick_score_num_;
  // for ( int uv = 0; uv < inflated_map_data_.size(); ++uv )
  for (int v = 0; v < height_; v += range_step_) {
    for (int u = 0; u < width_; u += range_step_) {
      const int uv = v * width_ + u;
      if (!inflated_map_data_[uv].first) {
        continue;
      }

      for (int i = 0; i < laserscan_samples_.size(); ++i) {
        const LaserScanSample& sample = laserscan_samples_[i];
        if (u + sample.min_x <= 1 || u + sample.max_x >= width_ - 1)
          continue;
        if (v + sample.min_y <= 1 || v + sample.max_y >= height_ - 1)
          continue;

        double sample_score = 0;
        if (quick_score_) {
          int object = 0;
          for (int point_index = 0; point_index < sample_size;
               point_index += step) {
            if (inflated_map_data_[sample.indices[point_index]
                                   + uv].second >= 0.5) {
              // if ( validPosition(uv, sample.indices[point_index]) )
              ++object;
            }
          }

          if (object <= quick_score_num_ / 2)
            continue;
        }
        count++;

        for (auto point_index : sample.indices)
          sample_score += inflated_map_data_[point_index + uv].second;
        sample_score /= static_cast<double>(sample_size);

        if (sample_score > score) {
          if (quick_score_) {
            int object = 0;
            for (int point_index = 0; point_index < sample_size;
                 point_index += step) {
              if (validPosition(uv, sample.indices[point_index]))
                ++object;
            }
            if (object <= quick_score_num_ / 2)
              continue;
          }

          score = sample_score;
          best_angle_ =  i * laserscan_anglar_step_ * RADIAN_PRE_DEGREE;
          best_position_ = micvision::Pixel(u, v);
        }
      }
    }
  }
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  double qx, qy, qz, qw;
  cout<< "best_position_[0] = "<<best_position_[0] << " best_position_[1] = " <<best_position_[1]<<" resolution_ = "<<resolution_<<endl;
  const double x = best_position_[0]*resolution_ + current_map_.getOriginX();
  const double y = best_position_[1]*resolution_ + current_map_.getOriginY();

  tf2::Quaternion q;
  q.setRPY(0, 0, best_angle_);            //
  qx = q.x();
  qy = q.y();
  qz = q.z();
  qw = q.w();

  // qz = cos(0) * cos(0) * sin(-best_angle_/2) - sin(0) * sin(0) * cos(-best_angle_/2);
  // qw = cos(0) * cos(0) * cos(-best_angle_/2) + sin(0) * sin(0) * sin(-best_angle_/2);

  //init_pose.header.stamp = ros::Time::now();
  rclcpp::Time now = this->get_clock()->now();

  init_pose.header.stamp = now;
  init_pose.header.frame_id = "map";

  init_pose.pose.pose.position.x = x;
  init_pose.pose.pose.position.y = y;
  init_pose.pose.pose.position.z = 0;

  init_pose.pose.pose.orientation.x = 0.0;
  init_pose.pose.pose.orientation.y = 0.0;
  init_pose.pose.pose.orientation.z = qz;
  init_pose.pose.pose.orientation.w = qw;

  init_pose.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.06853891945200942
                              };
  publisher_->publish(init_pose);

  RCLCPP_INFO(get_logger(),"Best score: %f, angle: %f, Best position: %f, %f, count: %d",
           score, best_angle_ / RADIAN_PRE_DEGREE, x, y, count);

    if(best_position_[0]!=0){
    rclcpp::shutdown();
  }
}

bool MicvisionLocalization::receiveLocalization(
  std_srvs::srv::Trigger_Request& req, std_srvs::srv::Trigger_Response& res) {
  init_pose_callback();
  res.success = true;
  res.message = "Localization success.";
  return true;

  /*
   *char fn[4096];
   *sprintf(fn, "/home/tyu/10101.txt");
   *FILE *fp = fopen(fn, "w");
   *for ( int i = 0; i < current_map_.getHeight(); i++ ) {
   *  for ( int j = 0; j < current_map_.getWidth(); j++ ) {
   *    fprintf(fp, "%d ", current_map_.getData(i, j));
   *  }
   *  fprintf(fp, "\n");
   *}
   *fclose(fp);
   */
}

// void MicvisionLocalization::getMap1(nav_msgs::srv::GetMap::Request::SharedPtr request_message, nav_msgs::srv::GetMap::Response::SharedPtr response_message) {
//     // uint structure_needs_at_least_one_member;
//     // request_message->structure_needs_at_least_one_member;
//     // response_message->map;
//     current_map_.update(response_message->map);
//     RCLCPP_INFO(get_logger(), "This is new GetMap functioin");

// }


void MicvisionLocalization::getMap() {
// bool MicvisionLocalization::getMap(nav_msgs::srv::GetMap::Request::SharedPtr request_message, nav_msgs::srv::GetMap::Response::SharedPtr response_message) {

      // new map comming?
  // if (!has_new_map_)
  //   return false;

  // if (!get_map_client_->isValid()) {
  //   RCLCPP_INFO(get_logger(),"get map client is invalid!");
  //   return false;
  // }

  //std::shared_ptr<nav_msgs::srv::GetMap::Response> response;
  //response->map;
  //std::shared_ptr<nav_msgs::srv::GetMap::Response> response;
  //response->map;
  //nav_msgs::srv::GetMap srv;
  //request_message->1;

  //response_message->map;
  //using GetMap_Response = struct nav_msgs::srv::GetMap_Response
  // if (!get_map_client_->call(srv)) {
  //   RCLCPP_INFO(get_logger(),"Could not get a map.");
  //   return false;
  // }
  //current_map_.update(response_message->map);

    while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      //return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    auto result = client->async_send_request(request);

  // auto node = std::make_shared<MicvisionLocalization>();
    // if (rclcpp::spin_until_future_complete(Node, result) ==
    // rclcpp::FutureReturnCode::SUCCESS)
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) 
    != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "got the /map_server/map");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /map_server/map");
  }

    current_map_.update(result.get()->map);

  RCLCPP_INFO(get_logger(),"Now inflation the static map.");
  cell_inflation_radius_ = inflation_radius_ / current_map_.getResolution();
  computeCaches();

  // uint structure_needs_at_least_one_member;
  // nav_msgs::srv::GetMap::Request::SharedPtr request_message;
  // request_message->structure_needs_at_least_one_member;
  // nav_msgs::srv::GetMap::Response::SharedPtr response_message;
  // response_message->map;
  // getMap1(request_message, response_message);

  // has_new_map_ = false;
  //return true;
}

void MicvisionLocalization::computeCaches() {
  cached_costs_ = new signed char* [cell_inflation_radius_ + 2];
  cached_distances_ = new double*[cell_inflation_radius_ + 2];

  for (unsigned int i = 0; i < cell_inflation_radius_ + 2; i++) {
    cached_costs_[i] = new signed char[cell_inflation_radius_ + 2];
    cached_distances_[i] = new double[cell_inflation_radius_ + 2];
    for (unsigned int j = 0; j < cell_inflation_radius_ + 2; j++) {
      double d = sqrt(static_cast<double>(i * i + j * j));
      cached_distances_[i][j] = d;
      d /= static_cast<double>(cell_inflation_radius_);
      if (d > 1) d = 1;
      cached_costs_[i][j] = (1.0 - d) * cost_obstacle_;
    }
  }
}

void MicvisionLocalization::inflateMap() {
  RCLCPP_INFO(get_logger(),"Infalting the map ...");
  const int map_size = current_map_.getSize();
  width_ = current_map_.getWidth();
  height_ = current_map_.getHeight();
  resolution_ = current_map_.getResolution();
  cout<<"width = "<<width_<<" height = "<<height_<<" resolution = "<<resolution_<<endl;

  if (inflation_markers_)
    delete[] inflation_markers_;
  inflation_markers_ = new unsigned char[map_size];
  memset(inflation_markers_, 0, map_size * sizeof(unsigned char));

  while (!inflation_queue_.empty())
    inflation_queue_.pop();

  for (int index = 0; index < map_size; index++) {
    if (current_map_.getData(index) > 0) {
      unsigned int x, y;
      current_map_.getCoordinates(x, y, index);
      enqueueObstacle(index, x, y);
    }
    /*
     *else if ( current_map_.getData(index) == -1 ) {
     *  inflation_markers_[index] = 1;
     *}
     */
  }

  int count = 0;
  while (!inflation_queue_.empty()) {
    const CellData cell = inflation_queue_.top();
    inflation_queue_.pop();

    unsigned int x, y;
    if (!current_map_.getCoordinates(x, y, cell.index))
      continue;

    if (x >= 1)
      enqueueObstacle(cell.index - 1, cell.x, cell.y);

    if (x < width_ - 1)
      enqueueObstacle(cell.index + 1, cell.x, cell.y);

    if (y >= 1)
      enqueueObstacle(cell.index - width_, cell.x, cell.y);

    if (y < current_map_.getHeight() - 1)
      enqueueObstacle(cell.index + width_, cell.x, cell.y);
    count++;
  }
  RCLCPP_INFO(get_logger(),"Inflated %i cells", count);

  inflated_map_data_.reserve(height_ * width_);
  for (int u = 0; u < height_; ++u) {
    for (int v = 0; v < width_; ++v) {
      const auto data = current_map_.getData(v, u);
      if (data >= 0 && data < 50) {
        inflated_map_data_.emplace_back(
          std::make_pair(true,
                         static_cast<double>(data) /
                         static_cast<double>(cost_obstacle_)));
      } else
        inflated_map_data_.emplace_back(
          std::make_pair(false,
                         static_cast<double>(data) /
                         static_cast<double>(cost_obstacle_)));
      /*
       *inflated_map_data_.emplace_back(
       *    std::make_pair(data >= 0 && data < 50, data));
       */
    }
  }
}


void MicvisionLocalization::enqueueObstacle(const unsigned int index,
                                            const unsigned int x,
                                            const unsigned int y) {
  unsigned int mx, my;
  if (!current_map_.getCoordinates(mx, my, index) ||
      inflation_markers_[index] != 0)
    return;

  const double d = distanceLookup(mx, my, x, y);
  if (d > cell_inflation_radius_)
    return;

  const CellData cell(d, index, x, y);
  inflation_queue_.push(cell);
  inflation_markers_[index] = 1;
  const signed char value = costLookup(mx, my, x, y);
  current_map_.setData(index, value);
}

inline double MicvisionLocalization::distanceLookup(
  const unsigned int mx, const unsigned int my,
  const unsigned int sx, const unsigned int sy) const {
  const unsigned int dx = abs(static_cast<int>(mx) - static_cast<int>(sx));
  const unsigned int dy = abs(static_cast<int>(my) - static_cast<int>(sy));

  if (dx > cell_inflation_radius_ + 1 || dy > cell_inflation_radius_ + 1) {
    RCLCPP_INFO(get_logger(),"Error in distanceLookup table! Asked for"
              " (%d, %d), but cell_inflation_radius_ is %d!",
              dx, dy, cell_inflation_radius_);
    return cell_inflation_radius_ + 1;
  }
  return cached_distances_[dx][dy];
}

inline signed char MicvisionLocalization::costLookup(
  const unsigned int mx, const unsigned int my,
  const unsigned int sx, const unsigned int sy) const {
  const unsigned int dx = abs(static_cast<int>(mx) - static_cast<int>(sx));
  const unsigned int dy = abs(static_cast<int>(my) - static_cast<int>(sy));

  if (dx > cell_inflation_radius_ + 1 || dy > cell_inflation_radius_ + 1) {
    RCLCPP_INFO(get_logger(),"Error in distanceLookup table! Asked for"
              " (%d, %d), but cell_inflation_radius_ is %d!",
              dx, dy, cell_inflation_radius_);
    return 0;
  }
  return cached_costs_[dx][dy];
}

// void MicvisionLocalization::debugAPosition(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) {  //const
//   // handleLaserScan();
//   //mtx.lock();
//   struct LaserScanSample sample = transformPointCloud(Eigen::Quaternionf(
//                                                  Eigen::AngleAxisf(pose.theta, Eigen::Vector3f::UnitZ())));
//   //mtx.unlock();
//   const double origin_x = current_map_.getOriginX();
//   const double origin_y = current_map_.getOriginY();
//   RCLCPP_INFO(get_logger(),"origin: [%f, %f]", origin_x, origin_y);

//   const int u = (pose.x - origin_x) / resolution_ + 1;
//   const int v = (pose.y - origin_y) / resolution_ + 1;

//   double score = 0;
//   RCLCPP_INFO(get_logger(),"u: %d, v:%d", u, v);

//   /*
//    *if ( v + sample.min_y <= 1 || v + sample.max_y >= height_ - 1 ||
//    *    u + sample.min_x <= 1 || u + sample.max_x >= width_ - 1 ) {
//    *  RCLCPP_INFO(get_logger(),"the laser is out of map's bound.");
//    *  return 0.0f;
//    *}
//    */

//   const int uv = v * width_ + u;

//   for (const auto point_index : sample.indices) {
//     const auto index = point_index + uv;
//     if (index >= 0 && index < inflated_map_data_.size())
//       score += inflated_map_data_[point_index + uv].second;
//   }

//   score /= static_cast<double>(sample.indices.size());

//   RCLCPP_INFO(get_logger(),"Position: [%f, %f], angle: %f, score: %f",
//             pose.x, pose.y, pose.theta, score);
//   current_position_score_ = score;

//   /*
//    *  RCLCPP_INFO(get_logger(),"u: %d, v: %d", u, v );
//    *  for ( auto point_index : sample.indices )
//    *    RCLCPP_INFO(get_logger(),"index: %d, score: %d", point_index,
//    *             static_cast<int>( inflated_map_data_[point_index + uv].second ));
//    *
//    *  RCLCPP_INFO(get_logger(),"\n\n\nnext is the best position.");
//    *  const LaserScanSample &best_sample =
//    *      laserscan_samples_[( best_angle_+M_PI )/laserscan_anglar_step_ + 1];
//    *  const int best_uv = best_position_[1] * width_ + best_position_[0];
//    *  for ( const auto index : best_sample.indices )
//    *    RCLCPP_INFO(get_logger(),"index: %d, score: %d", index,
//    *             static_cast<int>( inflated_map_data_[index + best_uv].second ));
//    */
// }

// void MicvisionLocalization::tracking() {
//   geometry_msgs::msg::TransformStamped transform1;
//   double r ,p, y;
//   int num = 0;
//   while (true && rclcpp::ok()) {     //ros2? // && ros::ok()
//     // TODO: first to get the current localization
//     if (big_angle_twist_) {
//       RCLCPP_INFO(get_logger(),"anglar twist big...");
//       num = 0;
//     } else {
//       try {
//         transform = tf_buffer_->lookupTransform(
//             map_frame_, robot_frame_,
//             tf2::TimePointZero);

//       } catch (tf2::TransformException & ex) {
//         RCLCPP_INFO(get_logger(),"Could not get robot position: %s", ex.what());
//       }
//       geometry_msgs::msg::PoseWithCovarianceStamped pose;
//       pose.x = transform1.transform.translation.x;
//       pose.y = transform1.transform.translation.x;

//       tf2::Quaternion q(
//               transform1.transform.rotation.x,
//               transform1.transform.rotation.y,
//               transform1.transform.rotation.z,
//               transform1.transform.rotation.w);
//     tf2::Matrix3x3 m(q);
//     m.getRPY(r, p, y);

//       pose.theta = y;

//       debugAPosition(pose);
//       RCLCPP_INFO(get_logger(),"current position score: %f", current_position_score_);
//       if (current_position_score_ < 0.5)
//         ++num;
//       else
//         num = 0;

//       if (num > 10) {
//         // init_pose_callback();
//         num = 0;
//       }
//     }

//     // ros::spinOnce();
//     // rate.sleep();
//   }
//   // score current localization
// }

//  void MicvisionLocalization::shutdown(){
//     if(best_position_[0]!=0){
//     rclcpp::shutdown();
//   }

// }

void add(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
          std::shared_ptr<std_srvs::srv::Trigger::Response> res){
  // MicvisionLocalization::init_pose_callback();
  res->success = true;
  res->message = "Localization success.";
//   res->sum = req->a + req->b;
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming req\na: %ld" " b: %ld",
//                 req->a, req->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back res:");     // [%ld]", (long int)res->sum);

}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MicvisionLocalization>();
//  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("static_map_node");
  // rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client =
  //   node->create_client<nav_msgs::srv::GetMap>("/map_server/map");



  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service =
  node->create_service<std_srvs::srv::Trigger>("start_localization", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to give response.");

  client = node->create_client<nav_msgs::srv::GetMap>("/map_server/map");
  cout<<"  "<<current_map_.getResolution()<<endl;
  cout<<"  "<<current_map_.getHeight()<<endl;
  cout<<"  "<<current_map_.getWidth()<<endl;

  // auto node = std::make_shared<MicvisionLocalization>();
    //  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr service =
    // node->create_service<nav_msgs::srv::GetMap>("/static_map", &add);
  //node->scoreASample(sample, u, v);
  //node->mapCallback(map);
  //node->scanCallback(scan);
  //node->validPosition(uv, index);
  node->getMap();
  node->inflateMap();
  node->init_pose_callback();
  // node->shutdown();

  // if(best_position_[0]!=0){
  //   rclcpp::shutdown();
  // }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
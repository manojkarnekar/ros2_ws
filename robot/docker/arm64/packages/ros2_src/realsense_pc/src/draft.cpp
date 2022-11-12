#include "realsense_pc/qr_scan.hpp"





class QR_code_detection : public rclcpp::Node
{
  public:
    QR_code_detection()
    : Node("QR_code_detection")
    { 
        this->declare_parameter<std::string>("parent_frame", "map");
        this->get_parameter("parent_frame", parent_frame_);
        this->declare_parameter<std::string>("child_frame", "pal");
        this->get_parameter("child_frame", child_frame_);


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        target_tf = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    }
    public:

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
        std::string parent_frame_;
        std::string child_frame_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> target_tf;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        geometry_msgs::msg::TransformStamped transformStamped;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        
        vector<float> pixel_to_point(const rs2::depth_frame& frame, pixel u, int range);
        vector<float> pixel_to_point_1(const rs2::depth_frame& frame, pixel u);
        void decode(cv::Mat &im, vector<decodedObject>&decodedObjects);
        void display(cv::Mat &im, vector<decodedObject>&decodedObjects, rs2::depth_frame depth);
        vector<float> minmax_idx(vector<float> v);
        void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset);
        vector<vector<string>> read(std::string read_csv);
        void send_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, double x, double y, double q_z, double q_w);
        geometry_msgs::msg::TransformStamped tf_sub(std::string parent_frame, std::string child_frame);




        float alpha;
        float dp_x, dp_y, dp_z, qx, qy, qz, qw;
        vector<float> px;
        vector<float> py;
        vector<float> pz;
        vector<float> ox;
        vector<float> oy;
        vector<float> oz;
        vector<float> ow;
        vector<float> PoseIndex;

};


vector<float> QR_code_detection::minmax_idx(vector<float> v)
{
  auto it = std::minmax_element(v.begin(), v.end());
	float min = *it.first;
	// float max = *it.second;
  float i = it.first-v.begin();

  vector<float> vec {};
  vec.push_back(min);
  vec.push_back(i);
  return vec;
}

vector<float> QR_code_detection::pixel_to_point(const rs2::depth_frame& frame, pixel u, int range)
{
    vector<vector<float>> pixs;
    vector<float> dist;
    float upixel[2];
    float upoint[3];
    upixel[0] = static_cast<float>(u.first);
    upixel[1] = static_cast<float>(u.second); 

    auto udist = frame.get_distance(static_cast<int>(upixel[0]), static_cast<int>(upixel[1]));

    // Deproject from pixel to point in 3D
    rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);

    float x = upixel[0];
    float y = upixel[1];


    
    for(int i = 0; i<range; i++)
      {
        
          upixel[0] = static_cast<float>(x+i);    
          upixel[1] = static_cast<float>(y+i);
          udist = frame.get_distance(static_cast<int>(upixel[0]), static_cast<int>(upixel[1]));
          if(udist!=0)
            {
              rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
              dist.push_back(upoint[2]);
              pixs.push_back({upixel[0], upixel[1]});
            }

          upixel[0] = static_cast<float>(x+i);    
          upixel[1] = static_cast<float>(y-i);
          udist = frame.get_distance(static_cast<int>(upixel[0]), static_cast<int>(upixel[1]));
          if(udist!=0)
            {
              rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
              dist.push_back(upoint[2]);
              pixs.push_back({upixel[0], upixel[1]});
            }

          upixel[0] = static_cast<float>(x-i);    
          upixel[1] = static_cast<float>(y+i);
          udist = frame.get_distance(static_cast<int>(upixel[0]), static_cast<int>(upixel[1]));
          if(udist!=0)
            {
              rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
              dist.push_back(upoint[2]);
              pixs.push_back({upixel[0], upixel[1]});
            }

          upixel[0] = static_cast<float>(x+i);         
          upixel[1] = static_cast<float>(y+i);
          udist = frame.get_distance(static_cast<int>(upixel[0]), static_cast<int>(upixel[1]));
          if(udist!=0)
            {
              rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
              dist.push_back(upoint[2]);
              pixs.push_back({upixel[0], upixel[1]});
            }        
          
      }

    vector<float> vec_ = minmax_idx(dist);
    // double val_ = vec_.at(0);
    int idx_ = vec_.at(1);
    udist = dist[idx_];
    upixel[0] = pixs[idx_][0];
    upixel[1] = pixs[idx_][1];
    rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);

    return {upoint[0], upoint[1], upoint[2]} ;
 
}


vector<float> QR_code_detection::pixel_to_point_1(const rs2::depth_frame& frame, pixel u)
{
    vector<vector<float>> pixs;
    vector<float> dist;
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


// Find and decode barcodes and QR codes
void QR_code_detection::decode(cv::Mat &im, vector<decodedObject>&decodedObjects)
{
  
  ImageScanner scanner;
 
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0);
  scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
  Mat imGray;
  cvtColor(im, imGray, COLOR_BGR2GRAY);
 
  zbar::Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
 
  int n = scanner.scan(image);
  
  for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;
    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();
 
    // cout << "Type : " << obj.type << endl;
    // cout << "Data : " << obj.data << endl << endl;
 
    for(int i = 0; i< symbol->get_location_size(); i++)
    {
      obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
    }
 
    decodedObjects.push_back(obj);
  }
}

void QR_code_detection::display(cv::Mat &im, vector<decodedObject>&decodedObjects, rs2::depth_frame depth)
{
  for(int i = 0; i < decodedObjects.size(); i++)
  {
    vector<Point> points = decodedObjects[i].location;
    vector<Point> hull;
 
    if(points.size() > 4)
      convexHull(points, hull);
    else
      hull = points;
 
    int n = hull.size();
 
    for(int j = 0; j < n; j++)
    {
      line(im, hull[j], hull[ (j+1) % n], Scalar(255,0,0), 3);
    }
    
    cv::circle(im, cv::Point((hull[0], hull[1])), 5, cv::Scalar(0,255,0),-1, 1,0);
    cv::circle(im, cv::Point((hull[1], hull[0])), 5, cv::Scalar(0,255,0),-1, 1,0);
    cv::circle(im, cv::Point((hull[2], hull[3])), 5, cv::Scalar(0,255,0),-1, 1,0);
    cv::circle(im, cv::Point((hull[3], hull[2])), 5, cv::Scalar(0,255,0),-1, 1,0);

    // left mid point
    cv::circle(im, cv::Point(((hull[0]+hull[1])/2.0, (hull[1]+hull[0])/2.0)), 5, cv::Scalar(0,255,0),-1, 1,0);
    // upper mid point
    cv::circle(im, cv::Point(((hull[0]+hull[1])/2.0, (hull[2]+hull[3])/2.0)), 5, cv::Scalar(0,255,0),-1, 1,0);
    // right mid point
    cv::circle(im, cv::Point(((hull[2]+hull[3])/2.0, (hull[3]+hull[2])/2.0)), 5, cv::Scalar(0,255,0),-1, 1,0);
    // lower mid point
    cv::circle(im, cv::Point(((hull[1]+hull[0])/2.0, (hull[3]+hull[2])/2.0)), 5, cv::Scalar(0,255,0),-1, 1,0);

    int num = 9;
    vector<cv::Point> pt(num);
    vector<float> x(num), y(num), z(num);
    vector<vector<float>> point(num);

    //mid point
    // pt.push_back((hull[0]+hull[1]+hull[2]+hull[3])/4.0 , (hull[1]+hull[0] + hull[3]+hull[2])/4.0);
    pt[0] = ((hull[0]+hull[1]+hull[2]+hull[3])/4.0 , (hull[1]+hull[0] + hull[3]+hull[2])/4.0);
    // int im_x = image.get_width() / 2;
    // int im_y = image.get_height() / 2;

    pt[1] = (hull[0], hull[1]);
    pt[2] = (hull[1], hull[0]);
    pt[3] = (hull[2], hull[3]);
    pt[4] = (hull[3], hull[2]);
    pt[5] = ((hull[0]+hull[1])/2.0, (hull[1]+hull[0])/2.0);
    pt[6] = ((hull[2]+hull[3])/2.0, (hull[3]+hull[2])/2.0);
    pt[7] = ((hull[0]+hull[1])/2.0, (hull[2]+hull[3])/2.0);
    pt[8] = ((hull[1]+hull[0])/2.0, (hull[3]+hull[2])/2.0);
    // pt[9] = ((hull[0]+hull[1]+hull[2]+hull[3]+hull[1]+hull[0] + hull[3]+hull[2])/8.0, (hull[2]+hull[3]+hull[3]+hull[2])/4.0); //right edge and midpoint centre
    // pt[10] = ((hull[0]+hull[1]+hull[1]+hull[0])/4.0, (hull[0]+hull[1]+hull[2]+hull[3]+hull[1]+hull[0] + hull[3]+hull[2])/8.0); //left edge and midpoint centre
    // pt[11] = ((hull[0]+hull[1]+hull[2]+hull[3])/4.0, (hull[0]+hull[1]+hull[2]+hull[3]+hull[1]+hull[0] + hull[3]+hull[2])/8.0);  // upper edge and midpoint centre
    // pt[12] = ((hull[0]+hull[1]+hull[2]+hull[3]+hull[1]+hull[0] + hull[3]+hull[2])/8.0, (hull[1]+hull[0]+hull[3]+hull[2])/4.0);  // midpoint and lower edge center
    
    int count_1 = 1;

    x[0] = pixel_to_point(depth, {pt[0].x, pt[0].y}, 30)[0];
    y[0] = pixel_to_point(depth, {pt[0].x, pt[0].y}, 30)[1];
    z[0] = pixel_to_point(depth, {pt[0].x, pt[0].y}, 30)[2];
    point[0] = {x[0], y[0], z[0]};

    for(int i = 1; i < num; i++)
    {
      if(i <= 6)
      {
        x[i] = pixel_to_point_1(depth, {pt[i].x, pt[i].y})[0];
        y[i] = pixel_to_point_1(depth, {pt[i].x, pt[i].y})[1];
        z[i] = pixel_to_point_1(depth, {pt[i].x, pt[i].y})[2];
      }

      if( i > 6)
      {
        x[i] = pixel_to_point(depth, {pt[i].x, pt[i].y}, 30)[0];
        y[i] = pixel_to_point(depth, {pt[i].x, pt[i].y}, 30)[1];
        z[i] = pixel_to_point(depth, {pt[i].x, pt[i].y}, 30)[2];
      }

      if(z[0] - 0.3 < z[i] && z[i] < z[0] + 0.3 && z[i]!=0)
      {
        point[count_1] = {x[i], y[i], z[i]};
        count_1 = count_1 + 1;
      }
    }

    for(int i = 0; i < num; i++)
    {
      cout << "x = " << x[i] << " y = " << y[i] << " z = " << z[i] << endl;
    }
    cout << count_1 << endl;

    vector<float> C(3);                                         //Center point of all points
    double tempsum = 0;
    int count = 0;
    for(int i=0; i<3; i++) {
        tempsum = 0;
        count = 0;
        for(int j=0; j < count_1; j++) {
            tempsum += point[j][i];
            count++;
            // cout << count << endl;            
            }
        C[i] = tempsum / count;
    }

    MatrixXd r0(count_1, 3);       // r0 = point - C
    for(int i = 0; i < count_1; i++)
    { 
      r0(i, 0) = point[i][0] - C[0];
      r0(i, 1) = point[i][1] - C[1];
      r0(i, 2) = point[i][2] - C[2];
    }

    // point.clear();

    MatrixXd inMat(count_1, 3);

    for(int i = 0; i < count_1; i++)
    {
      for(int j = 0; j < 3; j++)
      {
        inMat(i, j) = r0(i, j);
      }
    }

    BDCSVD<MatrixXd> svd(inMat, ComputeFullU | ComputeFullV);
 
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    auto sigma = svd.singularValues().asDiagonal().toDenseMatrix();


    MatrixXd nv(3, 1);
    for(int k = 0; k<3; k++)
    {
      nv(k, 0) = V(k, 2);
    }

    

    // if(nv(0, 0) > 0)
    // {
      alpha = (atan2(nv(2, 0), nv(0, 0)))*(180.0/3.14);

      // cout << nv(0, 0) << "  " << nv(1, 0) << "  " << nv(2, 0) << endl;
    // }
    // else
    // {
    //   alpha = (atan2(nv(2, 0), -nv(0, 0)))*(180.0/3.14);
    // }

    if(alpha < 0)
      { 
        alpha = alpha + 90;
      }
    else
        {
          alpha = alpha - 90;
        }

    float gamma = (atan2(nv(2, 0), nv(1, 0)))*(180.0/3.14);

    if(gamma < 0)
      {
        gamma = gamma + 90;
      }
    else
        {
          gamma = gamma - 90;
        }

    cout << "alpha = " << alpha << " gamma = " << gamma << endl;



  // cout << count_1 << endl;
  // for(int i = 0; i < num; i++)
  // {
  //   cout << "x[i] = " << x[i] << " y[i] = " << y[i] <<" z[i] = " << z[i] <<endl;
  // }
    
    vector<vector<string>> pose = read("qr_pose.csv");

    double tx, ty, tz, t_x, t_y, t_z;

    tx = x[0];
    ty = z[0];
    tz = 0.0;
  
    double alpha_1 = (180.0 - alpha)*3.14/180.0;    //correct

    // if(alpha_1<0){
    //   alpha_1 = 2*3.14159 - fabs(alpha_1);
    // }

    // tx = -tx;
    // ty = -ty;

    // t_x = tx*cos(alpha_1) + ty*sin(alpha_1);
    // t_y = -tx*sin(alpha_1) + ty*cos(alpha_1);
    // t_z = 0.0;
    

    double qx, qy, qz, qw, r_1, p_1, y_1, dp_x, dp_y, dp_z, q_x, q_y, q_z, q_w, p_x, p_y, p_z; 

    p_x = stof(pose[9][0]);
    p_y = stof(pose[9][1]);
    qx = stof(pose[9][3]);
    qy = stof(pose[9][4]);
    qz = stof(pose[9][5]);               // waypoints orientation in quaternion
    qw = stof(pose[9][6]);

    tf2::Quaternion q1(qx, qy, qz, qw);
    tf2::Matrix3x3 m(q1);
    m.getRPY(r_1, p_1, y_1);

    tf2::Quaternion q2;
    q2.setRPY(0, 0, y_1 + alpha_1);
    q_x = q2.x();                                
    q_y = q2.y();
    q_z = q2.z();
    q_w = q2.w();

    // geometry_msgs::msg::TransformStamped trans = tf_sub("base_footprint", "pal");


    // float t_x1 = trans.transform.translation.x;
    // float t_y1 = trans.transform.translation.y;

    float t_x1 = 125.0;

    if(alpha < 0)
    {
      float alpha_0 = fabs(alpha);
      t_x = z[0] + t_x1*sin(alpha_0);
      t_y =  x[0] + t_x1*cos(alpha_0); 
    }

    if(alpha >= 0)
    {
      t_x = z[0] + t_x1*sin(alpha);
      t_y = x[0] - t_x1*cos(alpha); 
    }   

    dp_x = t_x*cos(y_1) - t_y*sin(y_1) + p_x;
    dp_y = t_x*sin(y_1) + t_y*cos(y_1) + p_y;
    dp_z = 0.0;


    send_target_tf(this->now(), "map", "qr_code", dp_x, dp_y, q_z, q_w);

    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    rclcpp::Time now = this->get_clock()->now();

    init_pose.header.stamp = now;
    init_pose.header.frame_id = "map";

    init_pose.pose.pose.position.x = dp_x;
    init_pose.pose.pose.position.y = dp_y;
    init_pose.pose.pose.position.z = 0.0;

    init_pose.pose.pose.orientation.x = 0.0;
    init_pose.pose.pose.orientation.y = 0.0;
    init_pose.pose.pose.orientation.z = q_z;
    init_pose.pose.pose.orientation.w = q_w;

    init_pose.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,  0.0, 0.0, 0.0, 0.0, 0.06853891945200942
                                };
    publisher_->publish(init_pose);


    imshow("Results", im);
    waitKey(1);
 
}
}

void QR_code_detection::send_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, double x, double y, double q_z, double q_w)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = 0.0;

  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = q_z;
  tf_msg.transform.rotation.w = q_w;

  target_tf->sendTransform(tf_msg);
}


geometry_msgs::msg::TransformStamped QR_code_detection::tf_sub(std::string parent_frame, std::string child_frame)
{

try{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped = tf_buffer_->lookupTransform(
    parent_frame.c_str(), child_frame.c_str(),
    tf2::TimePointZero);

    return transformStamped;
}

catch (tf2::TransformException & ex) 
    {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            parent_frame_.c_str(), child_frame_.c_str(), ex.what());
    }
}

vector<vector<string>> QR_code_detection::read(std::string read_csv)
{
    ifstream file;
    vector<vector<string>> content;
    vector<string> row;
    string line, word;
            // file.open("/home/user/sar/docker/arm64/packages/ros2_src/core0_teach.csv");
            file.open(read_csv);
            file>>line;
            string str = line;
            stringstream ss(str);
            vector<string> v;
            if(file.is_open())
            {
            while(getline(file, line))
            {
                row.clear();
            stringstream str(line);
            
            while(getline(str, word, ','))
            row.push_back(word);
            content.push_back(row);
            }
            }
            else
            {
                cout<<"Could not open the file\n";
            }
    return content;

}

int main(int argc, char* argv[]) try
{
    rclcpp::init(argc, argv);

    QR_code_detection qr;
    rs2::pipeline p;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
    
    cv::Mat dMat_colored_bgr, col_frame_l_bgr, col_frame_l_bgra;
    p.start();

  while(rclcpp::ok())
  {
    
    rs2::frameset frames = p.wait_for_frames();
    auto colored_frame_l = frames.get_color_frame();
    auto depth = frames.get_depth_frame();
    cv::Mat dMat_colored_l = cv::Mat(colored_frame_l.get_height(), colored_frame_l.get_width(), CV_8UC3, (void*)colored_frame_l.get_data());

    cv::cvtColor(dMat_colored_l, col_frame_l_bgr, cv::COLOR_BGRA2BGR);

    vector<decodedObject> decodedObjects;

    qr.decode(col_frame_l_bgr, decodedObjects);
    qr.display(col_frame_l_bgr, decodedObjects, depth);
  }

  rclcpp::spin(std::make_shared<QR_code_detection>());
  rclcpp::shutdown();
  cv::destroyAllWindows();
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
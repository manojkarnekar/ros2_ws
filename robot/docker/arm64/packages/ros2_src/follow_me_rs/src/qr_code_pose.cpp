#include "follow_me_rs/qr_code_rs.hpp"



class QR_code_detection : public rclcpp::Node
{
  public:
    QR_code_detection()
    : Node("QR_code_detection")
    { 
        qr_pose_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("qr_pose", 10);
        img_pose_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("img_pose", 10);
        qr_data_pub = this->create_publisher<std_msgs::msg::String>("qr_data", 10);
    }
    public:
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr qr_pose_pub;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr img_pose_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_data_pub;
        
        bool pix_pcl(const rs2::depth_frame& frame, float x, float y, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub);
        void decode(cv::Mat &im, vector<decodedObject>&decodedObjects);
        void display(cv::Mat &im, vector<decodedObject>&decodedObjects, rs2::depth_frame depth);
        void Condition(cv::Mat &im, float x, float y, const rs2::depth_frame& depth, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub);
        vector<float> minmax_idx(vector<float> v);

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

bool QR_code_detection::pix_pcl(const rs2::depth_frame& frame, float x, float y, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub)
{

    std_msgs::msg::Float32MultiArray pose_msg;
    vector<vector<float>> pixs;
    vector<float> dist;
    float d = 0.0;
    float upixel[2];
    float upoint[3];

    upixel[0] = static_cast<float>(x);               //static_cast<int>(x);
    upixel[1] = static_cast<float>(y);               //static_cast<int>(y);  
    auto udist = frame.get_distance(static_cast<int>(upixel[0]), static_cast<int>(upixel[1]));

    rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);



    if(upoint[2]!=0)
    {

      for(int i = 0; i<15; i++)
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
          upixel[1] = static_cast<float>(y+i);
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
          upixel[1] = static_cast<float>(y-i);
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

    int r;
    if(r!=0)
    {
      d = upoint[2];
      r = 0;
    }

    if(d != 0.0 && upoint[2] < d + 0.5)
    {
      pose_msg.data.push_back(upoint[0]);
      pose_msg.data.push_back(upoint[1]);
      pose_msg.data.push_back(upoint[2]);
      pose_pub->publish(pose_msg);

      d = upoint[2];
    }
    float distance = sqrt(upoint[0]*upoint[0] + upoint[1]*upoint[1] + upoint[2]*upoint[2]);
    cout << "Distance between camera and qr code = " << distance << endl;
    cout << "x = " << upoint[0] << " y = " << upoint[1] << " z = " << upoint[2] <<endl;

    return true;
    }

    else
    {
        return false;
    }
 
}


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
 
    // Print type and data
    cout << "Type : " << obj.type << endl;
    cout << "Data : " << obj.data << endl << endl;

    auto std_message = std_msgs::msg::String();
    std_message.data = obj.data;

    qr_data_pub->publish(std_message);
 
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

    cv::Point pt;

    //mid point
    pt = ((hull[0]+hull[1]+hull[2]+hull[3])/4.0 , (hull[1]+hull[0] + hull[3]+hull[2])/4.0);


    float im_x = depth.get_width() / 2;
    float im_y = depth.get_height() / 2;

    Condition(im, pt.x, pt.y, depth, qr_pose_pub);
    
    Condition(im, im_x, im_y, depth, img_pose_pub);

}
    
  // Display results
  imshow("Results", im);
  waitKey(1);
 
}



void QR_code_detection::Condition(cv::Mat &im, float x, float y, const rs2::depth_frame& depth, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub)
{
    if (pix_pcl(depth, x, y, pose_pub) == true)
    {
        cv::circle(im, cv::Point(x , y), 5, cv::Scalar(0,255,0),-1, 1,0);
    }

    else
    {
        for(long int i=0; i<=15; i++)
        {
            if (pix_pcl(depth, x+i , y+i, pose_pub) == true)
            {
                cv::circle(im, cv::Point(x+i , y+i), 5, cv::Scalar(0,255,0),-1, 1,0);
            }

            else
            {
                if (pix_pcl(depth, x+i , y-i, pose_pub) == true)
                {
                    cv::circle(im, cv::Point(x+i , y-i), 5, cv::Scalar(0,255,0),-1, 1,0);
                }

                else
                {
                    if (pix_pcl(depth, x-i , y+i, pose_pub) == true)
                    {
                        cv::circle(im, cv::Point(x-i , y+i), 5, cv::Scalar(0,255,0),-1, 1,0);
                    }
                    else
                    {
                        if (pix_pcl(depth, x-i , y-i, pose_pub) == true)
                        {
                            cv::circle(im, cv::Point(x-i , y-i), 5, cv::Scalar(0,255,0),-1, 1,0);
                        }
                        else
                        {
                            cout<<"The Distance can not be computed by ZED2 PCL"<<endl;
                        }
                    }

                }
            }

        }

    }


}


int main(int argc, char* argv[]) try
{
    rclcpp::init(argc, argv);

    QR_code_detection qr;
    rs2::pipeline p;

    rs2::config cfg;
    // cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);
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
  cv::destroyAllWindows();
  rclcpp::shutdown();
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

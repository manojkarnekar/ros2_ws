#include "cv_localization/qr_code_pose.cpp"


void localization::tf_update()
{
try {
            transformStamped = tf_buffer_->lookupTransform(
            parent_frame_.c_str(), child_frame_.c_str(),
            tf2::TimePointZero);
            p_x = transformStamped.transform.translation.x;
            p_y = transformStamped.transform.translation.y;
            p_z = transformStamped.transform.translation.z;
            o_x = transformStamped.transform.rotation.x;
            o_y = transformStamped.transform.rotation.y;
            o_z = transformStamped.transform.rotation.z;
            o_w = transformStamped.transform.rotation.w;
        } 
    catch (tf2::TransformException & ex) 
    {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            parent_frame_.c_str(), child_frame_.c_str(), ex.what());
    }
    

//   if (lmData > 0 || rmData > 0)
//   {
    px.push_back(p_x);
    py.push_back(p_y);
    pz.push_back(p_z);
    ox.push_back(o_x);
    oy.push_back(o_y);
    oz.push_back(o_z);
    ow.push_back(o_w);
    // Wrap into a vector
    std::vector<std::pair<std::string, std::vector<float>>> vals = {{"px", px}, {"py", py}, {"pz", pz}, {"ox",ox}, {"oy",oy}, {"oz",oz}, {"ow",ow}};
    // Write the vector to CSV
    write_csv("core0_teach.csv", vals);
//   }
//   if (ipData == 1)
//   {
    int poseVecSize =px.size();
    PoseIndex.push_back(poseVecSize);
    cout<<"poseVecSize == "<<poseVecSize<<endl;
//   }
  std::vector<std::pair<std::string, std::vector<float>>> posevals = {{"PoseIndex", PoseIndex}};
  write_csv("core0_flash_pose.csv", posevals);
    
}

void localization::write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset)
{
    // Create an output filestream object
    std::ofstream myFile(filename);

     // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";
    
    // Send data to the stream
    for(int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for(int j = 0; j < dataset.size(); ++j)
        {
            myFile << dataset.at(j).second.at(i);
            if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }
    
    // Close the file
    myFile.close();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<localization>(); 
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
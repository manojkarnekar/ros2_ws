#include <iostream>
#include <queue>
#include <string>
#include <iterator>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <atomic>
#include <mutex>   
#include<cmath>  
#include <fstream>         
#include <condition_variable>
#include<algorithm>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>

#include "yolo_v2_class.hpp" 

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/opencv.hpp>

#include <sl/Camera.hpp>

#include "GLViewer.hpp"

constexpr float CONFIDENCE_THRESHOLD = 0.60;
constexpr float NMS_THRESHOLD = 0.4;
constexpr int NUM_CLASSES = 1;
constexpr int INFERENCE_SIZE = 416;

// colors for bounding boxes
const cv::Scalar colors[] = {
    {0, 255, 255},
    {255, 255, 0},
    {0, 255, 0},
    {255, 0, 0}
};
const auto NUM_COLORS = sizeof (colors) / sizeof (colors[0]);

std::mutex data_lock;
cv::Mat cur_frame;
std::vector<cv::Mat> result_vect;
std::atomic<bool> exit_flag, new_data;

class bbox_t_3d {
public:
    bbox_t bbox;
    sl::float3 coord;

    bbox_t_3d(bbox_t bbox_, sl::float3 coord_) {
        bbox = bbox_;
        coord = coord_;
    }
};

float getMedian(std::vector<float> &v) {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

std::vector<bbox_t_3d> getObjectDepth(std::vector<bbox_t> &bbox_vect, sl::Mat &xyzrgba) {
    sl::float4 out(NAN, NAN, NAN, NAN);
    bool valid_measure;
    int i, j;
    const int R_max = 4;

    std::vector<bbox_t_3d> bbox3d_vect;

    for (auto &it : bbox_vect) {

        int center_i = it.x + it.w * 0.5f, center_j = it.y + it.h * 0.5f;

        std::vector<float> x_vect, y_vect, z_vect;
        for (int R = 0; R < R_max; R++) {
            for (int y = -R; y <= R; y++) {
                for (int x = -R; x <= R; x++) {
                    i = center_i + x;
                    j = center_j + y;
                    xyzrgba.getValue<sl::float4>(i, j, &out, sl::MEM::GPU);
                    valid_measure = std::isfinite(out.z);
                    if (valid_measure) {
                        x_vect.push_back(out.x);
                        y_vect.push_back(out.y);
                        z_vect.push_back(out.z);
                    }
                }
            }
        }

        if (x_vect.size() * y_vect.size() * z_vect.size() > 0) {
            float x_med = getMedian(x_vect);
            float y_med = getMedian(y_vect);
            float z_med = getMedian(z_vect);

            bbox3d_vect.emplace_back(it, sl::float3(x_med, y_med, z_med));
        }
    }

    return bbox3d_vect;
}


void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix) {
    std::cout << "[Sample] ";
    if (err_code != sl::ERROR_CODE::SUCCESS)
        std::cout << "[Error] ";
    std::cout << msg_prefix << " ";
    if (err_code != sl::ERROR_CODE::SUCCESS) {
        std::cout << " | " << toString(err_code) << " : ";
        std::cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        std::cout << " " << msg_suffix;
    std::cout << std::endl;
}

std::vector<sl::uint2> cvt(const cv::Rect &bbox_in){
    std::vector<sl::uint2> bbox_out(4);
    bbox_out[0] = sl::uint2(bbox_in.x, bbox_in.y);
    bbox_out[1] = sl::uint2(bbox_in.x + bbox_in.width, bbox_in.y);
    bbox_out[2] = sl::uint2(bbox_in.x + bbox_in.width, bbox_in.y + bbox_in.height);
    bbox_out[3] = sl::uint2(bbox_in.x, bbox_in.y + bbox_in.height);
    return bbox_out;
}

float average(std::vector<float> &vi) {
     double sum = 0;

     for (int p:vi){
        sum = sum + p;
     }

     return (sum/vi.size());
    }


int main(int argc, char** argv) {
    // std::string names_file = "obj.names";
    // std::string cfg_file = "yolov4-tiny-custom.cfg";
    // std::string weights_file = "yolov4-tiny-custom_last.weights";

    std::string names_file = "coco.names";
    std::string cfg_file = "yolov4.cfg";
    std::string weights_file = "yolov4.weights";

    std::vector<std::string> class_names;
    {
        std::ifstream class_file(names_file);
        if (!class_file) {
            for (int i = 0; i < NUM_CLASSES; i++)
                class_names.push_back(std::to_string(i));
        } else {
            std::string line;
            while (std::getline(class_file, line))
                class_names.push_back(line);
        }
    }

    sl::Camera zed;
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    // init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed

    if (argc >= 2) {
        std::string zed_opt = argv[1];
        if (zed_opt.find(".svo") != std::string::npos)
            init_parameters.input.setFromSVOFile(zed_opt.c_str());
    }

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }

    zed.enablePositionalTracking();
    // Custom OD
    sl::ObjectDetectionParameters detection_parameters;
    detection_parameters.enable_tracking = true;
    // Let's define the model as custom box object to specify that the inference is done externally
    detection_parameters.detection_model = sl::DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
    returned_state = zed.enableObjectDetection(detection_parameters);

    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        print("enableObjectDetection", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    auto camera_config = zed.getCameraInformation().camera_configuration;
    // sl::Resolution image_size = zed.getCameraInformation().camera_resolution;
    sl::Resolution pc_resolution(std::min((int) camera_config.resolution.width, 720), std::min((int) camera_config.resolution.height, 404));
    auto camera_info = zed.getCameraInformation(pc_resolution).camera_configuration;

    sl::Mat left_sl, depth_image, point_cloud;

    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    sl::Objects objects;
    sl::Pose cam_w_pose;
    cam_w_pose.pose_data.setIdentity();

    auto net = cv::dnn::readNetFromDarknet(cfg_file, weights_file);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    // net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    // net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    auto output_names = net.getUnconnectedOutLayersNames();

    cv::Mat frame, blob;
    std::vector<cv::Mat> detections;

    exit_flag = false;

    std::ofstream myfile;
    // myfile.open ("coordinates.csv");

    int frame_count = 0;

    int csv_flag = 0;

    while (!exit_flag) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {

            zed.retrieveImage(left_sl, sl::VIEW::LEFT);
            zed.retrieveImage(depth_image, sl::VIEW::DEPTH);
            zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

            // Preparing inference
            cv::Mat left_cv_rgba = slMat2cvMat(left_sl);
            cv::cvtColor(left_cv_rgba, frame, cv::COLOR_BGRA2BGR);

            std::cout << frame.cols << " x " << frame.rows << std:: endl;

            cv::dnn::blobFromImage(frame, blob, 0.00392, cv::Size(INFERENCE_SIZE, INFERENCE_SIZE), cv::Scalar(), true, false, CV_32F);
            net.setInput(blob);
            net.forward(detections, output_names);

            std::cout << "         " << std::endl;
            std::cout << "******Frame No : ******" << frame_count << std::endl;

            // myfile << "Frame : " << frame_count << "\n";

            std::vector<int> indices[NUM_CLASSES];
            std::vector<cv::Rect> boxes[NUM_CLASSES];
            std::vector<cv::Point> circs[NUM_CLASSES];
            std::vector<float> scores[NUM_CLASSES];

            for (auto& output : detections) {
                const auto num_boxes = output.rows;
                for (int i = 0; i < num_boxes; i++) {
                    auto x = output.at<float>(i, 0) * frame.cols;
                    auto y = output.at<float>(i, 1) * frame.rows;
                    auto width = output.at<float>(i, 2) * frame.cols;
                    auto height = output.at<float>(i, 3) * frame.rows;
                    cv::Rect rect(x - width / 2, y - height / 2, width, height);
                    cv::Point circ(x, y);

                    for (int c = 0; c < NUM_CLASSES; c++) {
                        auto confidence = *output.ptr<float>(i, 5 + c);
                        if (confidence >= CONFIDENCE_THRESHOLD) {
                            boxes[c].push_back(rect);
                            circs[c].push_back(circ);
                            scores[c].push_back(confidence);
                        }
                    }
                }
            }

            for (int c = 0; c < NUM_CLASSES; c++)
                cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD, indices[c]);

            std::vector<sl::CustomBoxObjectData> objects_in;
            for (int c = 0; c < NUM_CLASSES; c++) {
                for (size_t i = 0; i < indices[c].size(); ++i) {
                    const auto color = colors[c % NUM_COLORS];

                    auto idx = indices[c][i];
                    const auto& rect = boxes[c][idx];
                    const auto& circ = circs[c][idx];
                    auto& rect_score = scores[c][idx];

                    // Fill the detections into the correct format
                    sl::CustomBoxObjectData tmp;
                    tmp.unique_object_id = sl::generate_unique_id();
                    tmp.probability = rect_score;
                    tmp.label = c;
                    tmp.bounding_box_2d = cvt(rect);
                    tmp.is_grounded = (c == 0); 

                    objects_in.push_back(tmp);

                    cv::rectangle(frame, rect, color, 3);
                    cv::circle(frame,circ,5,color,3);

                    sl::float4 point_cloud_value;

                    float distance;

                    // point_cloud.getValue(circ.x, circ.y, &point_cloud_value);
                    // distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
                    // printf("Distance to Camera at (%d, %d): %f mm\n", circ.x, circ.y, distance);

                    int val = 0;
                    do
                    {
                        point_cloud.getValue(circ.x+val, circ.y, &point_cloud_value);
                        distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
                        val = val + 1;
                    }while(isnan(distance)==true);

                    std::ostringstream label_ss;
                    label_ss << class_names[c] << ": " << std::fixed << std::setprecision(2) << scores[c][idx];
                    auto label = label_ss.str();

                    // myfile << label << "\n" ;

                    // myfile << point_cloud_value.x << "," << point_cloud_value.y << "," << point_cloud_value.z << "\n";

                    int baseline;
                    auto label_bg_sz = cv::getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
                    cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height - baseline - 10), cv::Point(rect.x +  + label_bg_sz.width, rect.y+20), color, cv::FILLED);
                    cv::putText(frame, label.c_str(), cv::Point(rect.x, rect.y - baseline - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));

                    std::cout << label.c_str() << std::endl;

                    std::string dist = std::to_string(distance/1000);
                    // cv::putText(frame, dist, cv::Point(rect.x, rect.y - baseline - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
                    // cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height - baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y), color, cv::FILLED);
                    cv::putText(frame, dist, cv::Point(rect.x, rect.y + baseline + 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));

                    // Horizontal line through mid of bounding box
                    cv::line(frame, cv::Point(0 , circ.y) , cv::Point(frame.cols , circ.y) ,cv::Scalar(0, 255, 0), 3, cv::LINE_4);

                    // float bb_left[3], bb_right[3];

                    std::vector<float> x_left_list, x_right_list;

                    std::vector<float> left_x, left_y, left_z;      //vectors to store x, y, z coordinates of left points of point cloud 
                    std::vector<float> right_x, right_y, right_z;   //vectors to store x, y, z coordinates of right points of point cloud

                    float d_left, d_right;

                    // Left most point of bounding box
                    // point_cloud.getValue(rect.x , circ.y , &point_cloud_value);
                    // distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                    val = 0;
                    int point_bb = 0;
                    do
                    {
                        point_bb = rect.x + val;
                        point_cloud.getValue(point_bb , circ.y , &point_cloud_value);
                        distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
                        val = val + 1;
                    }while(isnan(distance)==true);

                    d_left = distance;

                    // myfile << "Left bounding box\n"; 
                    // myfile << point_cloud_value.x << "," << point_cloud_value.y << "," << point_cloud_value.z << "\n";

                    cv::circle(frame, cv::Point(point_bb , circ.y),5, (255, 0, 255), 3, cv::FILLED);

                    // bb_left[0] = (point_cloud_value.x);
                    // bb_left[1] = (point_cloud_value.y);
                    // bb_left[2] = (point_cloud_value.z);

                    dist = std::to_string(distance/1000);
                    cv::putText(frame, dist, cv::Point(point_bb , circ.y),cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255));

                    // right most point of bounding box
                    // point_cloud.getValue(rect.x + rect.width , circ.y , &point_cloud_value);
                    // distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                    point_bb = 0;

                    do
                    {
                        point_bb = rect.x + rect.width + val;
                        point_cloud.getValue(point_bb , circ.y , &point_cloud_value);
                        distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
                        val = val - 1;
                    }while(isnan(distance)==true);

                    d_right = distance;

                    // myfile << "Right bounding box\n"; 
                    // myfile << point_cloud_value.x << "," << point_cloud_value.y << "," << point_cloud_value.z << "\n";

                    cv::circle(frame, cv::Point(point_bb , circ.y),5, (255, 0, 255), 3);
                    

                    // bb_right[0] = (point_cloud_value.x);
                    // bb_right[1] = (point_cloud_value.y);
                    // bb_right[2] = (point_cloud_value.z);

                    dist = std::to_string(distance/1000);
                    cv::putText(frame, dist, cv::Point(point_bb , circ.y),cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255));

                    int horizontal, left_count = 1;

                    // std::cout << rect.width << std::endl;

                    // // left points of bounding box
                    // myfile << "LHS points\n"; 
                    // myfile << "pc.x, pc.y, pc.z, distance, max - d, min - d, avg - d, d_lhs - d\n";

                    std::vector<float> left_distances, right_distances;

                    std::vector<float> left_dist_diff, right_dist_diff;

                    do{
                        horizontal = rect.x - (left_count * (0.2 * rect.width));

                        point_bb = 0;

                        do
                        {
                            point_bb = horizontal + val;
                            point_cloud.getValue(point_bb , circ.y , &point_cloud_value);
                            distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
                            val = val - 1;
                        }while(isnan(distance)==true);

                        left_distances.push_back(distance);

                        // left_dist_diff.push_back(d_left - distance);
                        
                        // myfile << point_cloud_value.x << "," << point_cloud_value.y << "," << point_cloud_value.z << "\n";

                        // x_left_list.push_back(point_cloud_value.z);

                        // myfile << d_left - distance << "\n";

                        // left_x.push_back(point_cloud_value.x);
                        // left_y.push_back(point_cloud_value.y);
                        // left_z.push_back(point_cloud_value.z);

                        // std::cout << "point cloud" << " " << point_cloud_value.x << " " << point_cloud_value.y << " " << point_cloud_value.z << std::endl;
                        cv::circle(frame, cv::Point(point_bb, circ.y),5, (0, 0, 255), 3);

                        dist = std::to_string(distance/1000);
                        cv::putText(frame, dist, cv::Point(point_bb, circ.y),cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 0));
                        left_count++;
                    }while(point_bb > 0);

                    // float max, min, avg;

                    // max = *max_element(left_distances.begin(), left_distances.end());
                    // min = *min_element(left_distances.begin(), left_distances.end());

                    // std::cout << "Max : " << min << "\nMin : " << max << std::endl; 
    
                    // avg = average(left_distances);

                    // max = max /1000;
                    // min = min / 1000;
                    // avg = avg / 1000;
                    // d_left = d_left / 1000;
                    // std::cout << "Average : " << avg << "\n"; 

                    // std::cout << "Size : " << left_x.size() << "   " << left_distances.size() << "    " <<  " \n " << std::endl;

                    // std::vector<std::vector<float>> left_table, right_table;

                    // for(int i=0; i<left_x.size(); i++)
                    // { 
                    //     std::vector<float> row;
                        
                    //     row.push_back(left_x[i] / 1000);
                    //     row.push_back(left_y[i] / 1000);
                    //     row.push_back(left_z[i] / 1000);
                    //     row.push_back(left_distances[i] / 1000);
                    //     row.push_back(max - left_distances[i] / 1000);
                    //     row.push_back(min - left_distances[i] / 1000);
                    //     row.push_back(avg - left_distances[i] / 1000);
                    //     row.push_back(d_left - left_distances[i] / 1000);

                    //     left_table.push_back(row);

                    //     // myfile << left_table[i][0] << "," << left_table[i][1] << "," << left_table[i][2] << "," << left_table[i][3] << "," << left_table[i][4] << "," << left_table[i][5] << "," << left_table[i][6] << "," << left_table[i][7] << std::endl;
                    // }

                    // std::cout << "left table size : " << left_table.size() << std::endl;

                    // std::cout << left_table[0][0] << "  " << left_table[0][1] << "  " << left_table[0][2] << "  " << left_table[0][3] << "  " << left_table[0][4] << "  " << left_table[0][5] << "  " << left_table[0][6] << "  " << left_table[0][7] << std::endl;

                    int right_count = 1;

                    // right points of bounding box
                    // myfile << "RHS points\n"; 
                    // myfile << "pc.x, pc.y, pc.z, distance, max - d, min - d, avg - d, d_rhs - d\n";

                    do{
                        horizontal = rect.x + rect.width + (right_count * (0.2 * rect.width));

                        point_bb = 0;

                        do
                        {
                            point_bb = horizontal + val;
                            point_cloud.getValue(point_bb , circ.y , &point_cloud_value);
                            distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
                            val = val + 1;
                        }while(isnan(distance)==true);

                        right_distances.push_back(distance);

                        // right_dist_diff.push_back(d_right - distance);

                        // myfile << d_right - distance << "\n";

                        // x_right_list.push_back(point_cloud_value.z);

                        // myfile << point_cloud_value.x << "," << point_cloud_value.y << "," << point_cloud_value.z << "\n";

                        cv::circle(frame, cv::Point(point_bb, circ.y),5, (0, 0, 255), 3);

                        // right_x.push_back(point_cloud_value.x);
                        // right_y.push_back(point_cloud_value.y);
                        // right_z.push_back(point_cloud_value.z);

                        dist = std::to_string(distance/1000);
                        cv::putText(frame, dist, cv::Point(point_bb, circ.y),cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 0));
                        right_count++;
                    }while(point_bb < frame.cols);

                    // cv::rectangle(frame, cv::Point((rect.x + rect.width/2) - (0.1* rect.width), (rect.y + rect.height/2) - (0.1* rect.height)), cv::Point((rect.x + rect.width/2) + (0.1* rect.width), (rect.y + rect.height/2) + (0.1* rect.height)), color);

                    // for points cluster around mid point of bounding box
                    int flag = 0;
                    float max = 0, min = 0;

                    float max_x = 0, max_y = 0, max_z = 0;
                    float min_x = 0, min_y = 0, min_z = 0;

                    for(float i = (rect.x + rect.width/2) - (0.1* rect.width); i < (rect.x + rect.width/2) + (0.1* rect.width); i=i+25)
                    {
                        for(float j = (rect.y + rect.height/2) - (0.2* rect.height); j < (rect.y + rect.height/2) + (0.2* rect.height); j=j+25)
                        {
                            point_cloud.getValue(i,j, &point_cloud_value);
                            distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                            if(isnan(distance)==false)
                            {
                                cv::circle(frame, cv::Point(i,j),5, (0, 0, 255), -1);
                                // std::cout << distance/1000 << std::endl;

                                if(flag == 0)
                                {
                                    max = distance;
                                    min = distance;

                                    max_x = point_cloud_value.x;
                                    max_y = point_cloud_value.y;
                                    max_z = point_cloud_value.z;

                                    min_x = point_cloud_value.x;
                                    min_y = point_cloud_value.y;
                                    min_z = point_cloud_value.z;

                                    // std::cout << "flag 0  " << max << "   " << min << std::endl;

                                    flag = 1;
                                }

                                if(max <= distance)
                                {
                                    max = distance;

                                    max_x = point_cloud_value.x;
                                    max_y = point_cloud_value.y;
                                    max_z = point_cloud_value.z;
                                }
                                if(min >= distance)
                                {
                                    min = distance;

                                    min_x = point_cloud_value.x;
                                    min_y = point_cloud_value.y;
                                    min_z = point_cloud_value.z;
                                }
                            }

                        }
                    }


                    std::cout << "Mid -> Max : " << max/1000 << " Min : " << min/1000 << std::endl;

                    std::cout << max_x << "  " << max_y << "  " << max_z << std::endl;
                    std::cout << min_x << "  " << min_y << "  " << min_z << std::endl;

                    dist = std::to_string(min/1000);
                    cv::putText(frame, dist, cv::Point(rect.x + rect.width/2 - 20 , rect.y + rect.height/2 - 10),cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cv::Scalar(0, 255, 0), 3);

                    // if(csv_flag == 0)
                    // {
                    //     myfile << min_x/1000 << "," << min_z/1000;

                    //     csv_flag = 1;
                    // }

                    myfile.open ("coordinates.csv");
                    myfile << min_x/1000 << "," << min_z/1000 << "\n";
                    myfile.close();

                    // dist = std::to_string(max/1000);
                    // cv::putText(frame, dist, cv::Point((rect.x + rect.width/2) - (0.1* rect.width), (rect.y + rect.height/2) - (0.2* rect.height) - 15),cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 0));
                    // dist = std::to_string(min/1000);
                    // cv::putText(frame, dist, cv::Point((rect.x + rect.width/2) - (0.1* rect.width), (rect.y + rect.height/2) + (0.2* rect.height) + 15),cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 0));

                    // for points cluster to the left of left point of bounding box
                    flag = 0;
                    max = 0, min = 0;

                    float left_max_x = 0, left_max_y = 0, left_max_z = 0;
                    float left_min_x = 0, left_min_y = 0, left_min_z = 0;

                    float left_cmin_i, left_cmin_j;

                    for(float i = rect.x - (4 * (0.2 * rect.width)); i < rect.x - (1 * (0.2 * rect.width)); i=i+25)
                    {
                        for(float j = (rect.y + rect.height/2) - (0.3* rect.height); j < (rect.y + rect.height/2) + (0.3* rect.height); j=j+25)
                        {
                            if(i > 0)
                            {
                                point_cloud.getValue(i,j, &point_cloud_value);
                                distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                                if(isnan(distance)==false)
                                {
                                    cv::circle(frame, cv::Point(i,j),5, (0, 0, 255), -1);
                                    // std::cout << distance/1000 << std::endl;

                                    if(flag == 0)
                                    {
                                        max = distance;
                                        min = distance;

                                        left_max_x = point_cloud_value.x;
                                        left_max_y = point_cloud_value.y;
                                        left_max_z = point_cloud_value.z;

                                        left_min_x = point_cloud_value.x;
                                        left_min_y = point_cloud_value.y;
                                        left_min_z = point_cloud_value.z;

                                        left_cmin_i = i;
                                        left_cmin_j = j;

                                        // std::cout << "flag 0  " << max << "   " << min << std::endl;

                                        flag = 1;
                                    }

                                    if(max <= distance)
                                    {
                                        max = distance;

                                        left_max_x = point_cloud_value.x;
                                        left_max_y = point_cloud_value.y;
                                        left_max_z = point_cloud_value.z;
                                    }
                                    if(min >= distance)
                                    {
                                        min = distance;

                                        left_min_x = point_cloud_value.x;
                                        left_min_y = point_cloud_value.y;
                                        left_min_z = point_cloud_value.z;

                                        left_cmin_i = i;
                                        left_cmin_j = j;
                                    }
                                }
                            }
                        }
                    }

                    cv::circle(frame, cv::Point(left_cmin_i,left_cmin_j),10, (255, 0, 0), -1);

                    std::cout << "Left -> Max : " << max/1000 << " Min : " << min/1000 << std::endl;

                    std::cout << left_max_x << "  " << left_max_y << "  " << left_max_z << std::endl;
                    std::cout << left_min_x << "  " << left_min_y << "  " << left_min_z << std::endl;

                    // for points cluster to the right of right point of bounding box
                    flag = 0;
                    max = 0, min = 0;

                    float right_max_x = 0, right_max_y = 0, right_max_z = 0;
                    float right_min_x = 0, right_min_y = 0, right_min_z = 0;

                    float right_cmin_i, right_cmin_j;

                    for(float i = rect.x + rect.width + (1 * (0.2 * rect.width)); i < rect.x + rect.width + (4 * (0.2 * rect.width)); i=i+25)
                    {
                        for(float j = (rect.y + rect.height/2) - (0.3* rect.height); j < (rect.y + rect.height/2) + (0.3* rect.height); j=j+25)
                        {
                            if(i < frame.cols)
                            {
                                point_cloud.getValue(i,j, &point_cloud_value);
                                distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                                if(isnan(distance)==false)
                                {
                                    cv::circle(frame, cv::Point(i,j),5, (0, 0, 255), -1);
                                    // std::cout << distance/1000 << std::endl;

                                    if(flag == 0)
                                    {
                                        max = distance;
                                        min = distance;

                                        right_max_x = point_cloud_value.x;
                                        right_max_y = point_cloud_value.y;
                                        right_max_z = point_cloud_value.z;

                                        right_min_x = point_cloud_value.x;
                                        right_min_y = point_cloud_value.y;
                                        right_min_z = point_cloud_value.z;

                                        right_cmin_i = i;
                                        right_cmin_j = j;

                                        // std::cout << "flag 0  " << max << "   " << min << std::endl;

                                        flag = 1;
                                    }

                                    if(max <= distance)
                                    {
                                        max = distance;

                                        right_max_x = point_cloud_value.x;
                                        right_max_y = point_cloud_value.y;
                                        right_max_z = point_cloud_value.z;
                                    }
                                    if(min >= distance)
                                    {
                                        min = distance;

                                        right_min_x = point_cloud_value.x;
                                        right_min_y = point_cloud_value.y;
                                        right_min_z = point_cloud_value.z;

                                        right_cmin_i = i;
                                        right_cmin_j = j;
                                    }
                                }
                            }
                        }
                    }

                    cv::circle(frame, cv::Point(right_cmin_i,right_cmin_j),10, (255, 0, 0), -1);

                    std::cout << "Right -> Max : " << max/1000 << " Min : " << min/1000 << std::endl;

                    std::cout << right_max_x << "  " << right_max_y << "  " << right_max_z << std::endl;
                    std::cout << right_min_x << "  " << right_min_y << "  " << right_min_z << std::endl;

                    // for points cluster at left point of bounding box
                    flag = 0;
                    max = 0, min = 0;

                    float left_bb_max_x = 0, left_bb_max_y = 0, left_bb_max_z = 0;
                    float left_bb_min_x = 0, left_bb_min_y = 0, left_bb_min_z = 0;

                    float left_bbmin_i, left_bbmin_j;

                    for(float j = (rect.y + rect.height/2) - (0.4* rect.height); j < (rect.y + rect.height/2) + (0.4* rect.height); j=j+25)
                    {
                        point_cloud.getValue(rect.x,j, &point_cloud_value);
                        distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                        if(isnan(distance)==false)
                        {
                            cv::circle(frame, cv::Point(rect.x,j),5, (0, 0, 255), -1);
                            // std::cout << distance/1000 << std::endl;

                            if(flag == 0)
                            {
                                max = distance;
                                min = distance;

                                left_bb_max_x = point_cloud_value.x;
                                left_bb_max_y = point_cloud_value.y;
                                left_bb_max_z = point_cloud_value.z;

                                left_bb_min_x = point_cloud_value.x;
                                left_bb_min_y = point_cloud_value.y;
                                left_bb_min_z = point_cloud_value.z;

                                left_bbmin_i = rect.x;
                                left_bbmin_j = j;

                                // std::cout << "flag 0  " << max << "   " << min << std::endl;

                                flag = 1;
                            }

                            if(max <= distance)
                            {
                                max = distance;

                                left_bb_max_x = point_cloud_value.x;
                                left_bb_max_y = point_cloud_value.y;
                                left_bb_max_z = point_cloud_value.z;
                            }
                            if(min >= distance)
                            {
                                min = distance;

                                left_bb_min_x = point_cloud_value.x;
                                left_bb_min_y = point_cloud_value.y;
                                left_bb_min_z = point_cloud_value.z;

                                left_bbmin_i = rect.x;
                                left_bbmin_j = j;
                            }
                        }
                    }

                    cv::circle(frame, cv::Point(left_bbmin_i,left_bbmin_j),10, (255, 0, 0), -1);

                    std::cout << "Left BB -> Max : " << max/1000 << " Min : " << min/1000 << std::endl;

                    std::cout << left_bb_max_x << "  " << left_bb_max_y << "  " << left_bb_max_z << std::endl;
                    std::cout << left_bb_min_x << "  " << left_bb_min_y << "  " << left_bb_min_z << std::endl;


                    // for points cluster at right point of bounding box
                    flag = 0;
                    max = 0, min = 0;

                    float right_bb_max_x = 0, right_bb_max_y = 0, right_bb_max_z = 0;
                    float right_bb_min_x = 0, right_bb_min_y = 0, right_bb_min_z = 0;

                    float right_bbmin_i, right_bbmin_j;

                    for(float j = (rect.y + rect.height/2) - (0.4* rect.height); j < (rect.y + rect.height/2) + (0.4* rect.height); j=j+25)
                    {
                        point_cloud.getValue(rect.x + rect.width , j, &point_cloud_value);
                        distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                        if(isnan(distance)==false)
                        {
                            cv::circle(frame, cv::Point(rect.x + rect.width ,j),5, (0, 0, 255), -1);
                            // std::cout << distance/1000 << std::endl;

                            if(flag == 0)
                            {
                                max = distance;
                                min = distance;

                                right_bb_max_x = point_cloud_value.x;
                                right_bb_max_y = point_cloud_value.y;
                                right_bb_max_z = point_cloud_value.z;

                                right_bb_min_x = point_cloud_value.x;
                                right_bb_min_y = point_cloud_value.y;
                                right_bb_min_z = point_cloud_value.z;

                                right_bbmin_i = rect.x + rect.width;
                                right_bbmin_j = j;

                                // std::cout << "flag 0  " << max << "   " << min << std::endl;

                                flag = 1;
                            }

                            if(max <= distance)
                            {
                                max = distance;

                                right_bb_max_x = point_cloud_value.x;
                                right_bb_max_y = point_cloud_value.y;
                                right_bb_max_z = point_cloud_value.z;
                            }
                            if(min >= distance)
                            {
                                min = distance;

                                right_bb_min_x = point_cloud_value.x;
                                right_bb_min_y = point_cloud_value.y;
                                right_bb_min_z = point_cloud_value.z;

                                right_bbmin_i = rect.x + rect.width;
                                right_bbmin_j = j;
                            }
                        }
                    }
                    
                    cv::circle(frame, cv::Point(right_bbmin_i,right_bbmin_j),10, (255, 0, 0), -1);

                    std::cout << "Right BB -> Max : " << max/1000 << " Min : " << min/1000 << std::endl;

                    std::cout << right_bb_max_x << "  " << right_bb_max_y << "  " << right_bb_max_z << std::endl;
                    std::cout << right_bb_min_x << "  " << right_bb_min_y << "  " << right_bb_min_z << std::endl;


                    float left_min_distance, right_min_distance;

                    left_min_distance = sqrt(((left_max_x - left_bb_max_x) * (left_max_x - left_bb_max_x)) + ((left_max_y - left_bb_max_y) * (left_max_y - left_bb_max_y)) + ((left_max_z - left_bb_max_z) * (left_max_z - left_bb_max_z)));

                    right_min_distance = sqrt(((right_max_x - right_bb_max_x) * (right_max_x - right_bb_max_x)) + ((right_max_y - right_bb_max_y) * (right_max_y - right_bb_max_y)) + ((right_max_z - right_bb_max_z) * (right_max_z - right_bb_max_z)));

                    std::cout << "left distance : " << left_min_distance/1000 << std::endl;

                    std::cout << "right distance : " << right_min_distance/1000 << std::endl;

                    cv::line(frame, cv::Point(left_bbmin_i, left_bbmin_j), cv::Point(left_cmin_i, left_cmin_j), (0, 0, 255), 2, cv::LINE_AA);
                    dist = std::to_string(left_min_distance/1000);
                    cv::putText(frame, dist, cv::Point(left_cmin_i, left_cmin_j),cv::FONT_HERSHEY_COMPLEX_SMALL, 2, cv::Scalar(0, 255, 0),2);

                    cv::line(frame, cv::Point(right_bbmin_i, right_bbmin_j), cv::Point(right_cmin_i, right_cmin_j), (0, 0, 255), 2, cv::LINE_AA);
                    dist = std::to_string(right_min_distance/1000);
                    cv::putText(frame, dist, cv::Point(right_cmin_i, right_cmin_j),cv::FONT_HERSHEY_COMPLEX_SMALL, 2, cv::Scalar(0, 255, 0),2);


                    // max = *max_element(right_distances.begin(), right_distances.end());
                    // min = *min_element(right_distances.begin(), right_distances.end());

                    // std::cout << "Max : " << min << "\nMin : " << max << std::endl; 
    
                    // avg = average(right_distances);
                    // // std::cout << "Average : " << avg << "\n";

                    // max = max /1000;
                    // min = min / 1000;
                    // avg = avg / 1000;
                    // d_right = d_right / 1000;

                    // std::cout << "Size : " << right_x.size() << "   " << right_distances.size() << "    " <<  " \n " << std::endl;

                    // for(int i=0; i<right_x.size(); i++)
                    // { 
                    //     std::vector<float> row;
                        
                    //     row.push_back(right_x[i] / 1000);
                    //     row.push_back(right_y[i] / 1000);
                    //     row.push_back(right_z[i] / 1000);
                    //     row.push_back(right_distances[i] / 1000);
                    //     row.push_back(max - right_distances[i] / 1000);
                    //     row.push_back(min - right_distances[i] / 1000);
                    //     row.push_back(avg - right_distances[i] / 1000);
                    //     row.push_back(d_right - right_distances[i] / 1000);

                    //     right_table.push_back(row);

                    //     // myfile << right_table[i][0] << "," << right_table[i][1] << "," << right_table[i][2] << "," << right_table[i][3] << "," << right_table[i][4] << "," << right_table[i][5] << "," << right_table[i][6] << "," << right_table[i][7]  << std::endl;
                    // }

                    // std::cout << "right table size : " << right_table.size() << std::endl;

                    // std::cout << right_table[0][0] << "  " << right_table[0][1] << "  " << right_table[0][2] << "  " << right_table[0][3] << "  " << right_table[0][4] << "  " << right_table[0][5] << "  " << right_table[0][6] << "  " << right_table[0][7] << "  " << std::endl;

                    // std::vector<int> lhs_dists, rhs_dists;

                    // std::cout << left_points.size() << "   " << right_points.size() << std::endl; 

                    // int temp;

                    // for(int i = 0; i < left_x.size(); i++)
                    // {
                    //     // std::cout << left_points[i][0] << " " << left_points[i][1] << " " << left_points[i][2] << " " << std::endl;
                    //     // temp = sqrt((int)((bb_left[0] - left_points[i][0]) ^ 2) + (int)((bb_left[1] - left_points[i][1]) ^ 2) + (int)((bb_left[2] - left_points[i][2]) ^ 2));
                    //     // temp = sqrt(pow((bb_left[0] - left_points[i][0]),2) + pow((bb_left[1] - left_points[i][1]),2) + pow((bb_left[2] - left_points[i][2]),2));
                    //     temp = sqrt(left_x*left_x + left_y*left_y + left_z*left_z);
                    //     left_distances.push_back(temp);

                    //     // std::cout << "Left Distance :" << temp << std::endl;
                    // }

                    // for(int i = 0; i < right_x.size() ; i++)
                    // {
                    //     // std::cout << right_points[i][0] << " " << right_points[i][1] << " " << right_points[i][2] << " " << std::endl;
                    //     // temp = sqrt((int)((right_points[i][0] - bb_right[0]) ^ 2) + (int)((right_points[i][1] - bb_right[1]) ^ 2) + (int)((right_points[i][2] - bb_right[2]) ^ 2));
                    //     // temp = sqrt(pow((right_points[i][0] - bb_right[0]),2) + pow((right_points[i][1] - bb_right[1]),2) + pow((right_points[i][2] - bb_right[2]),2));
                    //     temp = sqrt(right_x*right_x + right_y*right_y + right_z*right_z);
                    //     right_distances.push_back(temp);

                    //     // std::cout << "Right Distance :" << temp << std::endl;
                    // }

                    left_distances.clear();
                    right_distances.clear();

                    // left_x.clear();
                    // left_y.clear();
                    // left_z.clear();
                    // right_x.clear();
                    // right_y.clear();
                    // right_z.clear();
                }
            }
            zed.ingestCustomBoxObjects(objects_in);

                    // std::cout << cropped.cols << " x " << cropped.rows << std::endl;

                    

            cv::imshow("Objects", frame);

            // cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

            //         cv::GaussianBlur(frame, frame, cv::Size(3,3), 0);

            //         cv::Mat edges;
            //         cv::Canny(frame, edges, 50, 300, 5, false);
            // cv::imshow("Canny edge", edges);
            
            frame_count++;

            int key = cv::waitKey(10); 
            if (key == 27 || key == 'q') exit_flag = true;

            zed.retrieveObjects(objects, objectTracker_parameters_rt);

        }
    }
    // myfile.close();
    return 0;
}
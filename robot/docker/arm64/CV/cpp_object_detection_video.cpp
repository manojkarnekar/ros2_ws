#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace dnn;
using namespace std;

int main(int argc, char **argv)
{
    std::string names_file = "obj.names";
    std::string weights_file = "yolov4-custom_last.weights";
    std::string cfg_file = "yolov4-custom.cfg";
    std::string video_path = "bed1.mp4";

    if(argc > 1)
    {
        // cout << "More arguments" << endl;
        names_file = argv[1];
        weights_file = argv[2];
        cfg_file = argv[3];
        video_path = argv[4];
    }

    std::vector<std::string> classes;
    std::ifstream file(names_file);
    std::string line;
    while (std::getline(file, line)) {
        classes.push_back(line);
    }

    int count = 0;
 
    Net net = readNetFromDarknet(cfg_file, weights_file);

    net.setPreferableBackend(DNN_BACKEND_CUDA);
    net.setPreferableTarget(DNN_TARGET_CUDA);
 
    DetectionModel model = DetectionModel(net);
    model.setInputParams(1 / 255.0, Size(416, 416), Scalar(), true);

    VideoCapture video_capture(video_path);

    std::vector<int> classIds;
    std::vector<float> scores;
    std::vector<Rect> boxes;

    while (video_capture.isOpened())
    {
        Mat frame;

        bool Success = video_capture.read(frame);
        std::string frame_num;

        frame_num = "Frame " + std::to_string(count);

        cout << frame_num << endl;

        putText(frame, frame_num, Point(15,30), cv::FONT_HERSHEY_SIMPLEX, 1,
                    Scalar(0, 255, 0), 2);

        model.detect(frame, classIds, scores, boxes, 0.6, 0.4);

        for (int i = 0; i < classIds.size(); i++) {
            rectangle(frame, boxes[i], Scalar(0, 255, 0), 2);
    
            char text[100];
            snprintf(text, sizeof(text), "%s: %.2f", classes[classIds[i]].c_str(), scores[i]);
            putText(frame, text, Point(boxes[i].x, boxes[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 1,
                    Scalar(0, 255, 0), 2);

            cout << classes[classIds[i]].c_str() << endl;

            
        }

        if(Success == true)
        {
            imshow("video frame", frame);
        }
        else
        {
            cout << "Video frame not available" << endl;
            break;
        }

        int key;
        key = waitKey(20);
        // wait 20 ms between successive frames and break the loop if key q is pressed
        if(key == 'q')
        {
            cout << "Key Q is pressed. Video is stopping" << endl;
            destroyAllWindows();
            break;
        }
        count = count + 1;
    }

    video_capture.release();        //Release the video capture object
    // destroyAllWindows();

    return 0;
}
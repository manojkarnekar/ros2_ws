#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
 
using namespace cv;
using namespace dnn;
using namespace std;
 
int main()
{
    Mat img = imread("dog.jpg");
 
    std::vector<std::string> classes;
    std::ifstream file("coco.names");
    std::string line;
    while (std::getline(file, line)) {
        classes.push_back(line);
    }
 
    Net net = readNetFromDarknet("yolov4.cfg", "yolov4.weights");
 
    DetectionModel model = DetectionModel(net);
    model.setInputParams(1 / 255.0, Size(416, 416), Scalar(), true);
 
    std::vector<int> classIds;
    std::vector<float> scores;
    std::vector<Rect> boxes;
    model.detect(img, classIds, scores, boxes, 0.6, 0.4);
 
    for (int i = 0; i < classIds.size(); i++) {
        rectangle(img, boxes[i], Scalar(0, 255, 0), 2);
 
        char text[100];
        snprintf(text, sizeof(text), "%s: %.2f", classes[classIds[i]].c_str(), scores[i]);
        putText(img, text, Point(boxes[i].x, boxes[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 1,
                Scalar(0, 255, 0), 2);
        cout << text << endl;
    }
 
    imshow("Image", img);
    waitKey(0);
    destroyAllWindows();
 
    return 0;
}
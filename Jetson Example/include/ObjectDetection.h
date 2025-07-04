#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include <opencv4/opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>

struct DetectedObject
{
    cv::Rect bbox;
    float confidence;
    int classId;

};

class ObjectDetection {
public:
    
    ObjectDetection();  // Constructor
    std::vector<DetectedObject> decodeOutputs(const std::vector<float>& output1, const std::vector<float>& output2);


private:
    float sigmoid(float x);
    void processDetections(const std::vector<float>& output, int gridH, int gridW, int maskIndex, std::vector<DetectedObject>& detections);

    const std::vector<std::string> labels = {"MobileGoal", "RedRing", "BlueRing"};
    const std::vector<std::vector<int>> yolo_masks = {{3, 4, 5}, {0, 1, 2}};
    const std::vector<std::pair<float, float>> yolo_anchors = {{10, 14}, {23, 27},   {37, 58},
                                                               {81, 82}, {135, 169}, {344, 319}};
    
    const float confidenceThreshold = 0.3; // Default 0.3
    const std::vector<float> object_thresholds = {0.5, 0.5, 0.5}; // Default {0.5, 0.5, 0.5}
    const float nms_threshold = 0.5; 
    const int img_width = 320;
    const int img_height = 320;
    const int raw_width = 640;
    const int raw_height = 480;


};

#endif // OBJECT_DETECTION_H

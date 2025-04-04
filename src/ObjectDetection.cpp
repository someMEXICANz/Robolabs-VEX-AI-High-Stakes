#include <ObjectDetection.h>


ObjectDetection::ObjectDetection() 
{
}

std::vector<DetectedObject> ObjectDetection::decodeOutputs(const std::vector<float> &output1, const std::vector<float> &output2)
{
    std::vector<DetectedObject> Detections;
    processDetections(output1,10,10,0,Detections);
    processDetections(output2,20,20,1,Detections);

    // Apply Non-Max Suppression (NMS)
    std::vector<int> indices;
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;

    for (const auto& det : Detections) 
    {
        boxes.push_back(det.bbox);
        confidences.push_back(det.confidence);
    }

    // Apply NMS (thresholds can be adjusted)
    cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);

    std::vector<DetectedObject> finalDetections;
    for (int idx : indices) 
    {
        finalDetections.push_back(Detections[idx]);
    }

    return finalDetections;  // Return only non-overlapping detections







    return Detections;
}

float ObjectDetection::sigmoid(float x) 
{
    return 1.0f / (1.0f + exp(-x));
}

void ObjectDetection::processDetections(const std::vector<float>& output, int gridH, int gridW, int maskIndex, std::vector<DetectedObject>& detections) 
{
    int numAnchors = 3;
    int numAttributes = 8; // X, Y, W, H, Confidence, Class1, Class2, Class3

    float scale_x = raw_width / img_width;
    float scale_y = raw_height / img_height;

    std::vector<int> mask = yolo_masks[maskIndex]; // Select correct anchor mask

    for (int h = 0; h < gridH; h++) {
        for (int w = 0; w < gridW; w++) {
            for (int anchorIdx = 0; anchorIdx < numAnchors; anchorIdx++) {
                int anchor = mask[anchorIdx];  
                float anchor_w = yolo_anchors[anchor].first;
                float anchor_h = yolo_anchors[anchor].second;

        
                int confidenceIndex = ((anchorIdx * numAttributes + 4) * gridH + h) * gridW + w;
                float confidence = sigmoid(output[confidenceIndex]); 

                if (confidence < confidenceThreshold) continue; 

               
                int class1Index = ((anchorIdx * numAttributes + 5) * gridH + h) * gridW + w;
                int class2Index = ((anchorIdx * numAttributes + 6) * gridH + h) * gridW + w;
                int class3Index = ((anchorIdx * numAttributes + 7) * gridH + h) * gridW + w;
;
                float exp_sum = exp(output[class1Index]) + exp(output[class2Index]) + exp(output[class3Index]);
                float class1 = exp(output[class1Index]) / exp_sum;
                float class2 = exp(output[class2Index]) / exp_sum;
                float class3 = exp(output[class3Index]) / exp_sum;

                // Get best class
                std::vector<float> class_probs = {class1, class2, class3};
                int best_class = std::distance(class_probs.begin(), std::max_element(class_probs.begin(), class_probs.end()));
                
                float best_confidence = class_probs[best_class];
                if ((best_confidence ) < object_thresholds[best_class]) continue;

                int xIndex = ((anchorIdx * numAttributes + 0) * gridH + h) * gridW + w;
                int yIndex = ((anchorIdx * numAttributes + 1) * gridH + h) * gridW + w;
                int wIndex = ((anchorIdx * numAttributes + 2) * gridH + h) * gridW + w;
                int hIndex = ((anchorIdx * numAttributes + 3) * gridH + h) * gridW + w;

                float x_raw = output[xIndex];
                float y_raw = output[yIndex];
                float w_raw = output[wIndex];
                float h_raw = output[hIndex];

                // ✅ Apply YOLO decoding (grid-scale positioning)
                float x_center = (sigmoid(x_raw) + w) * (raw_width / gridW);
                float y_center = (sigmoid(y_raw) + h) * (raw_height / gridH);
                float width = exp(w_raw) * anchor_w * scale_x;
                float height = exp(h_raw) * anchor_h * scale_y;

                float x = x_center - width / 2.0f;
                float y = y_center - height / 2.0f;

                // ✅ Store valid detection
                cv::Rect box(x, y, width, height);
                DetectedObject Detected;
                Detected.bbox = box;
                Detected.confidence = confidence;
                Detected.classId = best_class;
                detections.push_back(Detected);
            }
        }
    }
}




























































































































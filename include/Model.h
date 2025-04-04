#ifndef MODEL_H
#define MODEL_H

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <numeric>


class Model 
{
public:
    // Constructor and Destructor
    Model();                                                                                // Constructor || Done
    ~Model();                                                                               // Destructor  || Done
    void runInference();
    // CPU Buffers                                                    
    std::vector<float>  inferInput,
                        inferOutput1,
                        inferOutput2;
    


private:
    // TensorRT engine-related objects
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IExecutionContext* context;
    nvinfer1::IRuntime* runtime;
    cudaStream_t stream;
    cudaError_t err;
    bool debug;


    const std::string& onnxPath = "highstakes_lite.onnx";
    const std::string& enginePath = "highstakes_lite.trt";

    int32_t inputIndex;
    int32_t outputIndex1;
    int32_t outputIndex2;
    
    size_t  inputSize = 1, 
            outputSize1 = 1, 
            outputSize2 = 1;

    bool hasFP16 = false;
    bool hasINT8 = false;
    bool hasTF32 = false;

    
 
    //GPU Buffers
    std::vector<void*> buffers;
    
    bool loadEngine();                                  // Engine Loader   || Done
    bool buildEngine();                                 // Engine Builder  || Done
    void allocateBuffers();                             // Aloocator       || Done
    void generateConfidenceHeatmap(const std::vector<float>& output, int gridH, int gridW, int anchorIndex);
    float sigmoid(float x);
   




  
};

#endif // MODEL_H

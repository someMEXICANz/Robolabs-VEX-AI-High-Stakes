#include <Model.h>
 
using namespace nvinfer1;
using namespace nvonnxparser;

// Logger for TensorRT
class Logger : public ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity != Severity::kINFO) {
            std::cerr << msg << std::endl;
        }
    }
};

Logger logger;

Model::Model()
{
    debug = false; 
    std::cerr << "Initializing Object Detection Model" << std::endl;
    bool enginebuilt = false;
    bool engineloaded = false;
    engineloaded = loadEngine();
    if (!engineloaded)
    {
        enginebuilt = buildEngine();
        engineloaded = loadEngine();
    }
    else
        enginebuilt = true;

    if(enginebuilt && engineloaded)
    {
        context = engine->createExecutionContext();
        if (!context) 
            std::cerr << "Failed to create execution context!" << std::endl;
        allocateBuffers();
    }
}

Model::~Model() 
{
    for (void* buffer : buffers) 
    {
        cudaFree(buffer);
    }
    context->destroy();
    engine->destroy();
    runtime->destroy();
    cudaStreamDestroy(stream);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool Model::loadEngine() 
{
    std::cerr << "Loading Engine file: " << enginePath << std::endl;
    std::ifstream engineFile(enginePath, std::ios::binary);
    if (!engineFile)
    {
        std::cerr << " Failed to load engine... " << std::endl;
        engine == nullptr;
        return false;
    }
    engineFile.seekg(0, std::ios::end);
    size_t engineSize = engineFile.tellg();
    engineFile.seekg(0, std::ios::beg);

    std::vector<char> engineData(engineSize);
    engineFile.read(engineData.data(), engineSize);
    engineFile.close();

    runtime = nvinfer1::createInferRuntime(logger);
    engine = runtime->deserializeCudaEngine(engineData.data(), engineSize, nullptr);

    return engine != nullptr;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool Model::buildEngine() 
{
    std::cerr << "Building engine this process may take a few minutes." << std::endl;
    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(logger);
    nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();

   

    const int32_t explicitBatch = 1;
    size_t maxWSsize = 1U << 28;
    builder->setMaxBatchSize(explicitBatch);
    config->setMaxWorkspaceSize(maxWSsize);

     unsigned int flags = 1U << static_cast<unsigned int>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(flags);
    
    std::cerr << "Loading and parsing onnx file:" << onnxPath << std::endl;
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, logger);
    if (!parser->parseFromFile(onnxPath.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR))) 
    {
        std::cerr << "Failed to parse ONNX file: " << onnxPath << std::endl;
        return false;
    }
    
    auto inputTensor = network->getInput(0);
    inputTensor->setDimensions(nvinfer1::Dims4{1, 3, 320, 320}); // Batch size 1
    
    engine = builder->buildEngineWithConfig(*network, *config);
    if (!engine) 
    {
        std::cerr << "Failed to build TensorRT engine!" << std::endl;
        return false;
    }

    nvinfer1::IHostMemory* serializedEngine = engine->serialize();
    std::ofstream engineFile(enginePath, std::ios::binary);
    engineFile.write(static_cast<const char*>(serializedEngine->data()), serializedEngine->size());
    std::cerr << "Saved built engine to disk" << std::endl;
    
    serializedEngine->destroy();
    parser->destroy();
    builder->destroy();
    config->destroy();

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Model::allocateBuffers() 
{
    // Create CUDA stream
    std::cerr << "Allocating Buffers" << std::endl;
    cudaStreamCreate(&stream);

    inputIndex = engine->getBindingIndex("000_net");
    outputIndex1 = engine->getBindingIndex("016_convolutional");
    outputIndex2 = engine->getBindingIndex("023_convolutional");
   
        // Validate binding indices
    if (inputIndex == -1 || outputIndex1 == -1 || outputIndex2 == -1) 
        throw std::runtime_error("Error: Invalid binding names!");

    // Query binding dimensions and calculate buffer sizes
    nvinfer1::Dims inputDims = engine->getBindingDimensions(inputIndex);
    nvinfer1::Dims outputDims1 = engine->getBindingDimensions(outputIndex1);
    nvinfer1::Dims outputDims2 = engine->getBindingDimensions(outputIndex2);

    // Calculate buffer sizes
    inputSize = sizeof(float);
    for (int i = 0; i < inputDims.nbDims; ++i) inputSize *= inputDims.d[i];
    outputSize1 = sizeof(float);
    for (int i = 0; i < outputDims1.nbDims; ++i) outputSize1 *= outputDims1.d[i];
    outputSize2 = sizeof(float);
    for (int i = 0; i < outputDims2.nbDims; ++i) outputSize2 *= outputDims2.d[i];
 
    // Resize buffers vector and allocate memory
    buffers.resize(engine->getNbBindings(), nullptr);
    inferInput.resize(inputSize / sizeof(float));
    inferOutput1.resize(outputSize1 / sizeof(float));
    inferOutput2.resize(outputSize2 / sizeof(float));

    cudaMalloc(&buffers[inputIndex], inputSize);
    cudaMalloc(&buffers[outputIndex1], outputSize1);
    cudaMalloc(&buffers[outputIndex2], outputSize2);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Model::runInference()
{
    if (buffers[inputIndex] == nullptr || buffers[outputIndex1] == nullptr || buffers[outputIndex2] == nullptr) 
        throw std::runtime_error ("Buffers are uninitialized! ");
   
    cudaMemcpyAsync(buffers[inputIndex], inferInput.data(), inputSize, cudaMemcpyHostToDevice, stream);

    context->enqueueV2(buffers.data(), stream, nullptr);
   
    cudaMemcpyAsync(inferOutput1.data(), buffers[outputIndex1], outputSize1, cudaMemcpyDeviceToHost, stream);
    cudaMemcpyAsync(inferOutput2.data(), buffers[outputIndex2], outputSize2, cudaMemcpyDeviceToHost, stream);

    cudaStreamSynchronize(stream);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


float Model::sigmoid(float x) 
{
    return 1.0f / (1.0f + exp(-x));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Model::generateConfidenceHeatmap(const std::vector<float>& output, int gridH, int gridW, int anchorIndex) 
{
    int numAnchors = 3;
    int numAttributes = 8;
    
    // Ensure anchor index is valid
    if (anchorIndex < 0 || anchorIndex > numAnchors) 
    {

        std::cerr << "Invalid anchor index!" << std::endl;
    }

    int confidenceChannel = anchorIndex * numAttributes + 4;  // Attribute 4 is confidence

    // Create a matrix to store confidence values
    cv::Mat heatmap(gridH, gridW, CV_32F);

    // Fill the heatmap with confidence values
    for (int h = 0; h < gridH; h++) 
    {
        for (int w = 0; w < gridW; w++) 
        {
            int index = ((confidenceChannel * gridH + h) * gridW) + w;
            heatmap.at<float>(h, w) = sigmoid(output[index]);  // Store confidence score
        }
    }

    // Normalize confidence values to 0-255 for visualization
    cv::Mat coloredHeatmap;
    cv::normalize(heatmap, heatmap, 0, 255, cv::NORM_MINMAX);
    heatmap.convertTo(heatmap, CV_8U);

    // Apply a colormap to enhance visualization
    
    cv::applyColorMap(heatmap, coloredHeatmap, cv::COLORMAP_JET);

    std::string window_suffix2 = " Grid (" + std::to_string(gridH) + "x" + std::to_string(gridW) + ")";
    std::string window_suffix1 = " Anchor " + std::to_string(anchorIndex);
    cv::imshow("Confidence Map" + window_suffix1 + window_suffix2, coloredHeatmap);
    
}







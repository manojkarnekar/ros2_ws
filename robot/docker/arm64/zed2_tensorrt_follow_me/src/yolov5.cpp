#include <iostream>
#include <chrono>
#include <cmath>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "utils.h"
#include "calibrator.h"
#include "GLViewer.hpp"

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
#include <cmath>  
#include <fstream>         
#include <condition_variable>
#include <algorithm>

#include <sl/Camera.hpp>

#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1

// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int CLASS_NUM = Yolo::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof (Yolo::Detection) / sizeof (float) + 1; // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;

static int get_width(int x, float gw, int divisor = 8) {
    return int(ceil((x * gw) / divisor)) * divisor;
}

static int get_depth(int x, float gd) {
    if (x == 1) return 1;
    int r = round(x * gd);
    if (x * gd - int(x * gd) == 0.5 && (int(x * gd) % 2) == 0) {
        --r;
    }
    return std::max<int>(r, 1);
}

ICudaEngine* build_engine(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt, float& gd, float& gw, std::string& wts_name) {
    INetworkDefinition* network = builder->createNetworkV2(0U);

    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor* data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
    assert(data);

    std::map<std::string, Weights> weightMap = loadWeights(wts_name);

    /* ------ yolov5 backbone------ */
    auto conv0 = convBlock(network, weightMap, *data,  get_width(64, gw), 6, 2, 1,  "model.0");
    assert(conv0);
    auto conv1 = convBlock(network, weightMap, *conv0->getOutput(0), get_width(128, gw), 3, 2, 1, "model.1");
    auto bottleneck_CSP2 = C3(network, weightMap, *conv1->getOutput(0), get_width(128, gw), get_width(128, gw), get_depth(3, gd), true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *bottleneck_CSP2->getOutput(0), get_width(256, gw), 3, 2, 1, "model.3");
    auto bottleneck_csp4 = C3(network, weightMap, *conv3->getOutput(0), get_width(256, gw), get_width(256, gw), get_depth(6, gd), true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *bottleneck_csp4->getOutput(0), get_width(512, gw), 3, 2, 1, "model.5");
    auto bottleneck_csp6 = C3(network, weightMap, *conv5->getOutput(0), get_width(512, gw), get_width(512, gw), get_depth(9, gd), true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *bottleneck_csp6->getOutput(0), get_width(1024, gw), 3, 2, 1, "model.7");
    auto bottleneck_csp8 = C3(network, weightMap, *conv7->getOutput(0), get_width(1024, gw), get_width(1024, gw), get_depth(3, gd), true, 1, 0.5, "model.8");
    auto spp9 = SPPF(network, weightMap, *bottleneck_csp8->getOutput(0), get_width(1024, gw), get_width(1024, gw), 5, "model.9");

    /* ------ yolov5 head ------ */
    auto conv10 = convBlock(network, weightMap, *spp9->getOutput(0), get_width(512, gw), 1, 1, 1, "model.10");

    auto upsample11 = network->addResize(*conv10->getOutput(0));
    assert(upsample11);
    upsample11->setResizeMode(ResizeMode::kNEAREST);
    upsample11->setOutputDimensions(bottleneck_csp6->getOutput(0)->getDimensions());

    ITensor * inputTensors12[] = {upsample11->getOutput(0), bottleneck_csp6->getOutput(0)};
    auto cat12 = network->addConcatenation(inputTensors12, 2);
    auto bottleneck_csp13 = C3(network, weightMap, *cat12->getOutput(0), get_width(1024, gw), get_width(512, gw), get_depth(3, gd), false, 1, 0.5, "model.13");
    auto conv14 = convBlock(network, weightMap, *bottleneck_csp13->getOutput(0), get_width(256, gw), 1, 1, 1, "model.14");

    auto upsample15 = network->addResize(*conv14->getOutput(0));
    assert(upsample15);
    upsample15->setResizeMode(ResizeMode::kNEAREST);
    upsample15->setOutputDimensions(bottleneck_csp4->getOutput(0)->getDimensions());

    ITensor * inputTensors16[] = {upsample15->getOutput(0), bottleneck_csp4->getOutput(0)};
    auto cat16 = network->addConcatenation(inputTensors16, 2);

    auto bottleneck_csp17 = C3(network, weightMap, *cat16->getOutput(0), get_width(512, gw), get_width(256, gw), get_depth(3, gd), false, 1, 0.5, "model.17");

    /* ------ detect ------ */
    IConvolutionLayer* det0 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW {
        1, 1 }, weightMap["model.24.m.0.weight"], weightMap["model.24.m.0.bias"]);
    auto conv18 = convBlock(network, weightMap, *bottleneck_csp17->getOutput(0), get_width(256, gw), 3, 2, 1, "model.18");
    ITensor * inputTensors19[] = {conv18->getOutput(0), conv14->getOutput(0)};
    auto cat19 = network->addConcatenation(inputTensors19, 2);
    auto bottleneck_csp20 = C3(network, weightMap, *cat19->getOutput(0), get_width(512, gw), get_width(512, gw), get_depth(3, gd), false, 1, 0.5, "model.20");
    IConvolutionLayer* det1 = network->addConvolutionNd(*bottleneck_csp20->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW {
        1, 1 }, weightMap["model.24.m.1.weight"], weightMap["model.24.m.1.bias"]);
    auto conv21 = convBlock(network, weightMap, *bottleneck_csp20->getOutput(0), get_width(512, gw), 3, 2, 1, "model.21");
    ITensor * inputTensors22[] = {conv21->getOutput(0), conv10->getOutput(0)};
    auto cat22 = network->addConcatenation(inputTensors22, 2);
    auto bottleneck_csp23 = C3(network, weightMap, *cat22->getOutput(0), get_width(1024, gw), get_width(1024, gw), get_depth(3, gd), false, 1, 0.5, "model.23");
    IConvolutionLayer* det2 = network->addConvolutionNd(*bottleneck_csp23->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW {
        1, 1 }, weightMap["model.24.m.2.weight"], weightMap["model.24.m.2.bias"]);

    auto yolo = addYoLoLayer(network, weightMap, "model.24", std::vector<IConvolutionLayer*>{det0, det1, det2});
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));

    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize(16 * (1 << 20)); // 16MB
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2* calibrator = new Int8EntropyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif

    std::cout << "Building engine, please wait for a while..." << std::endl;
    ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto& mem : weightMap) {
        free((void*) (mem.second.values));
    }

    return engine;
}

//v6.0
ICudaEngine* build_engine_p6(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt, float& gd, float& gw, std::string& wts_name) {
    INetworkDefinition* network = builder->createNetworkV2(0U);

    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor* data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
    assert(data);

    std::map<std::string, Weights> weightMap = loadWeights(wts_name);

    /* ------ yolov5 backbone------ */
    auto conv0 = convBlock(network, weightMap, *data,  get_width(64, gw), 6, 2, 1,  "model.0");
    auto conv1 = convBlock(network, weightMap, *conv0->getOutput(0), get_width(128, gw), 3, 2, 1, "model.1");
    auto c3_2 = C3(network, weightMap, *conv1->getOutput(0), get_width(128, gw), get_width(128, gw), get_depth(3, gd), true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *c3_2->getOutput(0), get_width(256, gw), 3, 2, 1, "model.3");
    auto c3_4 = C3(network, weightMap, *conv3->getOutput(0), get_width(256, gw), get_width(256, gw), get_depth(6, gd), true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *c3_4->getOutput(0), get_width(512, gw), 3, 2, 1, "model.5");
    auto c3_6 = C3(network, weightMap, *conv5->getOutput(0), get_width(512, gw), get_width(512, gw), get_depth(9, gd), true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *c3_6->getOutput(0), get_width(768, gw), 3, 2, 1, "model.7");
    auto c3_8 = C3(network, weightMap, *conv7->getOutput(0), get_width(768, gw), get_width(768, gw), get_depth(3, gd), true, 1, 0.5, "model.8");
    auto conv9 = convBlock(network, weightMap, *c3_8->getOutput(0), get_width(1024, gw), 3, 2, 1, "model.9");
    auto c3_10 = C3(network, weightMap, *conv9->getOutput(0), get_width(1024, gw), get_width(1024, gw), get_depth(3, gd), true, 1, 0.5, "model.10");
    auto sppf11 = SPPF(network, weightMap, *c3_10->getOutput(0), get_width(1024, gw), get_width(1024, gw), 5, "model.11");

    /* ------ yolov5 head ------ */
    auto conv12 = convBlock(network, weightMap, *sppf11->getOutput(0), get_width(768, gw), 1, 1, 1, "model.12");
    auto upsample13 = network->addResize(*conv12->getOutput(0));
    assert(upsample13);
    upsample13->setResizeMode(ResizeMode::kNEAREST);
    upsample13->setOutputDimensions(c3_8->getOutput(0)->getDimensions());
    ITensor * inputTensors14[] = {upsample13->getOutput(0), c3_8->getOutput(0)};
    auto cat14 = network->addConcatenation(inputTensors14, 2);
    auto c3_15 = C3(network, weightMap, *cat14->getOutput(0), get_width(1536, gw), get_width(768, gw), get_depth(3, gd), false, 1, 0.5, "model.15");

    auto conv16 = convBlock(network, weightMap, *c3_15->getOutput(0), get_width(512, gw), 1, 1, 1, "model.16");
    auto upsample17 = network->addResize(*conv16->getOutput(0));
    assert(upsample17);
    upsample17->setResizeMode(ResizeMode::kNEAREST);
    upsample17->setOutputDimensions(c3_6->getOutput(0)->getDimensions());
    ITensor * inputTensors18[] = {upsample17->getOutput(0), c3_6->getOutput(0)};
    auto cat18 = network->addConcatenation(inputTensors18, 2);
    auto c3_19 = C3(network, weightMap, *cat18->getOutput(0), get_width(1024, gw), get_width(512, gw), get_depth(3, gd), false, 1, 0.5, "model.19");

    auto conv20 = convBlock(network, weightMap, *c3_19->getOutput(0), get_width(256, gw), 1, 1, 1, "model.20");
    auto upsample21 = network->addResize(*conv20->getOutput(0));
    assert(upsample21);
    upsample21->setResizeMode(ResizeMode::kNEAREST);
    upsample21->setOutputDimensions(c3_4->getOutput(0)->getDimensions());
    ITensor * inputTensors21[] = {upsample21->getOutput(0), c3_4->getOutput(0)};
    auto cat22 = network->addConcatenation(inputTensors21, 2);
    auto c3_23 = C3(network, weightMap, *cat22->getOutput(0), get_width(512, gw), get_width(256, gw), get_depth(3, gd), false, 1, 0.5, "model.23");

    auto conv24 = convBlock(network, weightMap, *c3_23->getOutput(0), get_width(256, gw), 3, 2, 1, "model.24");
    ITensor * inputTensors25[] = {conv24->getOutput(0), conv20->getOutput(0)};
    auto cat25 = network->addConcatenation(inputTensors25, 2);
    auto c3_26 = C3(network, weightMap, *cat25->getOutput(0), get_width(1024, gw), get_width(512, gw), get_depth(3, gd), false, 1, 0.5, "model.26");

    auto conv27 = convBlock(network, weightMap, *c3_26->getOutput(0), get_width(512, gw), 3, 2, 1, "model.27");
    ITensor * inputTensors28[] = {conv27->getOutput(0), conv16->getOutput(0)};
    auto cat28 = network->addConcatenation(inputTensors28, 2);
    auto c3_29 = C3(network, weightMap, *cat28->getOutput(0), get_width(1536, gw), get_width(768, gw), get_depth(3, gd), false, 1, 0.5, "model.29");

    auto conv30 = convBlock(network, weightMap, *c3_29->getOutput(0), get_width(768, gw), 3, 2, 1, "model.30");
    ITensor * inputTensors31[] = {conv30->getOutput(0), conv12->getOutput(0)};
    auto cat31 = network->addConcatenation(inputTensors31, 2);
    auto c3_32 = C3(network, weightMap, *cat31->getOutput(0), get_width(2048, gw), get_width(1024, gw), get_depth(3, gd), false, 1, 0.5, "model.32");

    /* ------ detect ------ */
    IConvolutionLayer* det0 = network->addConvolutionNd(*c3_23->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW {
        1, 1 }, weightMap["model.33.m.0.weight"], weightMap["model.33.m.0.bias"]);
    IConvolutionLayer* det1 = network->addConvolutionNd(*c3_26->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW {
        1, 1 }, weightMap["model.33.m.1.weight"], weightMap["model.33.m.1.bias"]);
    IConvolutionLayer* det2 = network->addConvolutionNd(*c3_29->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW {
        1, 1 }, weightMap["model.33.m.2.weight"], weightMap["model.33.m.2.bias"]);
    IConvolutionLayer* det3 = network->addConvolutionNd(*c3_32->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW {
        1, 1 }, weightMap["model.33.m.3.weight"], weightMap["model.33.m.3.bias"]);

    auto yolo = addYoLoLayer(network, weightMap, "model.33", std::vector<IConvolutionLayer*>{det0, det1, det2, det3});
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));

    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize(16 * (1 << 20)); // 16MB
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2* calibrator = new Int8EntropyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif

    std::cout << "Building engine, please wait for a while..." << std::endl;
    ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto& mem : weightMap) {
        free((void*) (mem.second.values));
    }

    return engine;
}

void APIToModel(unsigned int maxBatchSize, IHostMemory** modelStream, bool& is_p6, float& gd, float& gw, std::string& wts_name) {
    // Create builder
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine *engine = nullptr;
    if (is_p6) {
        engine = build_engine_p6(maxBatchSize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    } else {
        engine = build_engine(maxBatchSize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    }
    assert(engine != nullptr);

    // Serialize the engine
    (*modelStream) = engine->serialize();

    // Close everything down
    engine->destroy();
    builder->destroy();
    config->destroy();
}

void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize) {
    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof (float), cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof (float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

bool parse_args(int argc, char** argv, std::string& wts, std::string& engine, bool& is_p6, float& gd, float& gw) {
    if (argc < 4) return false;
    if (std::string(argv[1]) == "-s" && (argc == 5 || argc == 7)) {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
        auto net = std::string(argv[4]);
        if (net[0] == 'n') {
            gd = 0.33;
            gw = 0.25;
        } else if (net[0] == 's') {
            gd = 0.33;
            gw = 0.50;
        } else if (net[0] == 'm') {
            gd = 0.67;
            gw = 0.75;
        } else if (net[0] == 'l') {
            gd = 1.0;
            gw = 1.0;
        } else if (net[0] == 'x') {
            gd = 1.33;
            gw = 1.25;
        } else if (net[0] == 'c' && argc == 7) {
            gd = atof(argv[5]);
            gw = atof(argv[6]);
        } else {
            return false;
        }
        if (net.size() == 2 && net[1] == '6') {
            is_p6 = true;
        }
    } else if (std::string(argv[1]) == "-d") {
        engine = std::string(argv[2]);
    } else {
        return false;
    }
    return true;
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

sl::Mat left_sl, depth_image, point_cloud;
cv::Mat left_cv_rgb;

sl::float4 point_cloud_value;
float distance;
int val;
std::string dist;

void draw_mid_points(cv::Mat left_cv_rgb, cv::Rect r, sl::Mat point_cloud, int baseline, float arr[], int ch)
{
    int val;
    float distance; 
    sl::float4 point_cloud_value;

    if(ch == 0)
    {
        val = 0;
        do
        {
            point_cloud.getValue(r.x + r.width/2 + val, r.y + r.height/2 , &point_cloud_value);
            distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
            val = val + 1;
        }while(isnan(point_cloud_value.z)==true);
        
        cv::circle(left_cv_rgb, cv::Point(r.x + r.width/2 , r.y + r.height/2), 8, (0, 0, 255), cv::FILLED);

        arr[0] = point_cloud_value.x;
        arr[1] = point_cloud_value.y;
        arr[2] = point_cloud_value.z;
        arr[3] = distance;
    }

    if(ch == 1)
    {
        val = 0;
        int point_bb = 0;
        do
        {
            point_bb = r.x + val;
            point_cloud.getValue(point_bb , r.y + r.height/2 , &point_cloud_value);
            distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
            val = val + 1;
        }while(isnan(point_cloud_value.z)==true);

        cv::circle(left_cv_rgb, cv::Point(r.x , r.y + r.height/2), 8, (0, 0, 255), cv::FILLED);

        arr[0] = point_cloud_value.x;
        arr[1] = point_cloud_value.y;
        arr[2] = point_cloud_value.z;
        arr[3] = distance;
    }

    if(ch == 2)
    {
        val = 0;
        int point_bb = 0;
        do
        {
            point_bb = r.x + r.width - val;
            point_cloud.getValue(point_bb , r.y + r.height/2, &point_cloud_value);
            distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
            val = val + 1;
        }while(isnan(point_cloud_value.z)==true);

        cv::circle(left_cv_rgb, cv::Point(r.x + r.width, r.y + r.height/2), 8, (0, 0, 255), cv::FILLED);

        arr[0] = point_cloud_value.x;
        arr[1] = point_cloud_value.y;
        arr[2] = point_cloud_value.z;
        arr[3] = distance;
    }


}

void draw_points_cluster(cv::Mat left_cv_rgb, cv::Rect r, sl::Mat point_cloud, float cluster_arr[], int ch)
{
    float distance; 
    sl::float4 point_cloud_value;
    float max, min;
    int flag;

    if(ch == 0)
    {
        flag = 0;
        max = 0, min = 0;

        float max_x = 0, max_y = 0, max_z = 0;
        float min_x = 0, min_y = 0, min_z = 0;

        for(float i = (r.x + r.width/2) - (0.1* r.width); i < (r.x + r.width/2) + (0.1* r.width); i=i+25)
        {
            for(float j = (r.y + r.height/2) - (0.2* r.height); j < (r.y + r.height/2) + (0.2* r.height); j=j+25)
            {
                point_cloud.getValue(i,j, &point_cloud_value);
                distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                if(isnan(distance)==false)
                {
                    cv::circle(left_cv_rgb, cv::Point(i,j),5, (0, 0, 255), -1);

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

        cluster_arr[0] = min_x;
        cluster_arr[1] = min_y;
        cluster_arr[2] = min_z;
        cluster_arr[3] = min;
    }

    if(ch == 1)
    {
        flag = 0;
        max = 0, min = 0;

        float left_max_x = 0, left_max_y = 0, left_max_z = 0;
        float left_min_x = 0, left_min_y = 0, left_min_z = 0;

        float left_cmin_i, left_cmin_j;

        for(float i = r.x - (4 * (0.2 * r.width)); i < r.x - (1 * (0.2 * r.width)); i=i+25)
        {
            for(float j = (r.y + r.height/2) - (0.3* r.height); j < (r.y + r.height/2) + (0.3* r.height); j=j+25)
            {
                if(i > 0)
                {
                    point_cloud.getValue(i,j, &point_cloud_value);
                    distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                    if(isnan(distance)==false)
                    {
                        cv::circle(left_cv_rgb, cv::Point(i,j),5, (0, 0, 255), -1);
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

        cv::circle(left_cv_rgb, cv::Point(left_cmin_i,left_cmin_j),10, (255, 0, 0), -1);

        cluster_arr[0] = left_min_x;
        cluster_arr[1] = left_min_y;
        cluster_arr[2] = left_min_z;
        cluster_arr[3] = min;
    }

    if(ch == 2)
    {
        flag = 0;
        max = 0, min = 0;

        float right_max_x = 0, right_max_y = 0, right_max_z = 0;
        float right_min_x = 0, right_min_y = 0, right_min_z = 0;

        float right_cmin_i, right_cmin_j;

        for(float i = r.x + r.width + (1 * (0.2 * r.width)); i < r.x + r.width + (4 * (0.2 * r.width)); i=i+25)
        {
            for(float j = (r.y + r.height/2) - (0.3* r.height); j < (r.y + r.height/2) + (0.3* r.height); j=j+25)
            {
                if(i < left_cv_rgb.cols)
                {
                    point_cloud.getValue(i,j, &point_cloud_value);
                    distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                    if(isnan(distance)==false)
                    {
                        cv::circle(left_cv_rgb, cv::Point(i,j),5, (0, 0, 255), -1);
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

        cv::circle(left_cv_rgb, cv::Point(right_cmin_i,right_cmin_j),10, (255, 0, 0), -1);

        cluster_arr[0] = right_min_x;
        cluster_arr[1] = right_min_y;
        cluster_arr[2] = right_min_z;
        cluster_arr[3] = min;
    }

    if(ch == 3)
    {
        flag = 0;
        max = 0, min = 0;

        float left_bb_max_x = 0, left_bb_max_y = 0, left_bb_max_z = 0;
        float left_bb_min_x = 0, left_bb_min_y = 0, left_bb_min_z = 0;

        float left_bbmin_i, left_bbmin_j;
        
        for(float i = r.x - (1 * (0.05 * r.width)); i < r.x + (1 * (0.1 * r.width)); i=i+25)
        {
            for(float j = (r.y + r.height/2) - (0.4* r.height); j < (r.y + r.height/2) + (0.4* r.height); j=j+25)
            {
                if(i > 0)
                {
                    point_cloud.getValue(i,j, &point_cloud_value);
                    distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                    if(isnan(distance)==false)
                    {
                        cv::circle(left_cv_rgb, cv::Point(i,j),5, (0, 0, 255), -1);
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

                            left_bbmin_i = i;
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

                            left_bbmin_i = i;
                            left_bbmin_j = j;
                        }
                    }
                }
            }
        }

        cv::circle(left_cv_rgb, cv::Point(left_bbmin_i,left_bbmin_j),10, (255, 0, 0), -1);

        cluster_arr[0] = left_bb_min_x;
        cluster_arr[1] = left_bb_min_y;
        cluster_arr[2] = left_bb_min_z;
        cluster_arr[3] = min;
    }

    if(ch == 4)
    {
        flag = 0;
        max = 0, min = 0;

        float right_bb_max_x = 0, right_bb_max_y = 0, right_bb_max_z = 0;
        float right_bb_min_x = 0, right_bb_min_y = 0, right_bb_min_z = 0;

        float right_bbmin_i, right_bbmin_j;

        for(float i = r.x + r.width - (1 * (0.1 * r.width)); i < r.x + r.width + (1 * (0.05 * r.width)); i=i+25)
        {
            for(float j = (r.y + r.height/2) - (0.4* r.height); j < (r.y + r.height/2) + (0.4* r.height); j=j+25)
            {
                if(i < left_cv_rgb.cols)
                {
                    point_cloud.getValue(i, j, &point_cloud_value);
                    distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

                    if(isnan(distance)==false)
                    {
                        cv::circle(left_cv_rgb, cv::Point(i,j),5, (0, 0, 255), -1);
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

                            right_bbmin_i = i;
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

                            right_bbmin_i = i;
                            right_bbmin_j = j;
                        }
                    }
                }
            }
        }
        
        cv::circle(left_cv_rgb, cv::Point(right_bbmin_i,right_bbmin_j),10, (255, 0, 0), -1);

        cluster_arr[0] = right_bb_min_x;
        cluster_arr[1] = right_bb_min_y;
        cluster_arr[2] = right_bb_min_z;
        cluster_arr[3] = min;
    }
}
int main(int argc, char** argv) {

    std::string wts_name = "";
    std::string engine_name = "";
    bool is_p6 = false;
    float gd = 0.0f, gw = 0.0f;
    if (!parse_args(argc, argv, wts_name, engine_name, is_p6, gd, gw)) {
        std::cerr << "arguments not right!" << std::endl;
        std::cerr << "./yolov5 -s [.wts] [.engine] [n/s/m/l/x/n6/s6/m6/l6/x6 or c/c6 gd gw]  // serialize model to plan file" << std::endl;
        std::cerr << "./yolov5 -d [.engine] [zed camera id / optional svo filepath]  // deserialize plan file and run inference" << std::endl;
        return -1;
    }

    // create a model using the API directly and serialize it to a stream
    if (!wts_name.empty()) {
        cudaSetDevice(DEVICE);
        IHostMemory * modelStream{ nullptr};
        APIToModel(BATCH_SIZE, &modelStream, is_p6, gd, gw, wts_name);
        assert(modelStream != nullptr);
        std::ofstream p(engine_name, std::ios::binary);
        if (!p) {
            std::cerr << "could not open plan output file" << std::endl;
            return -1;
        }
        p.write(reinterpret_cast<const char*> (modelStream->data()), modelStream->size());
        modelStream->destroy();
        return 0;
    }

    /// Opening the ZED camera before the model deserialization to avoid cuda context issue
    sl::Camera zed;
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
    // init_parameters.sdk_verbose = true;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    // init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed

    if (argc > 3) {
        std::string zed_opt = argv[3];
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
    detection_parameters.enable_mask_output = false; // designed to give person pixel mask
    detection_parameters.detection_model = sl::DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
    returned_state = zed.enableObjectDetection(detection_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        print("enableObjectDetection", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }
    auto camera_config = zed.getCameraInformation().camera_configuration;
    sl::Resolution pc_resolution(std::min((int) camera_config.resolution.width, 720), std::min((int) camera_config.resolution.height, 404));
    auto camera_info = zed.getCameraInformation(pc_resolution).camera_configuration;
    // Create OpenGL Viewer
    // GLViewer viewer;
    // viewer.init(argc, argv, camera_info.calibration_parameters.left_cam, true);
    // ---------

    // deserialize the .engine and run inference
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        return -1;
    }
    char *trtModelStream = nullptr;
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();

    // prepare input data ---------------------------
    static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
    static float prob[BATCH_SIZE * OUTPUT_SIZE];
    IRuntime* runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    IExecutionContext* context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    assert(engine->getNbBindings() == 2);
    void* buffers[2];
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof (float)));
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof (float)));
    // Create stream
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));

    assert(BATCH_SIZE == 1); // This sample only support batch 1 for now

    // sl::Mat left_sl, depth_image, point_cloud;
    // cv::Mat left_cv_rgb;
    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    sl::Objects objects;
    sl::Pose cam_w_pose;
    cam_w_pose.pose_data.setIdentity();

    std::ofstream myfile;

    int frame_count = 0;

    std::atomic<bool> exit_flag;
    exit_flag = false;

    while (!exit_flag) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {

            zed.retrieveImage(left_sl, sl::VIEW::LEFT);
            zed.retrieveImage(depth_image, sl::VIEW::DEPTH);
            zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

            // Preparing inference
            cv::Mat left_cv_rgba = slMat2cvMat(left_sl);
            cv::cvtColor(left_cv_rgba, left_cv_rgb, cv::COLOR_BGRA2BGR);

            std::cout << "         " << std::endl;
            std::cout << "******Frame No : " << frame_count << "******" << std::endl;

            // std::cout << left_cv_rgb.cols << " x " << left_cv_rgb.rows << std:: endl;

            if (left_cv_rgb.empty()) continue;
            cv::Mat pr_img = preprocess_img(left_cv_rgb, INPUT_W, INPUT_H); // letterbox BGR to RGB
            int i = 0;
            int batch = 0;
            for (int row = 0; row < INPUT_H; ++row) {
                uchar* uc_pixel = pr_img.data + row * pr_img.step;
                for (int col = 0; col < INPUT_W; ++col) {
                    data[batch * 3 * INPUT_H * INPUT_W + i] = (float) uc_pixel[2] / 255.0;
                    data[batch * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float) uc_pixel[1] / 255.0;
                    data[batch * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float) uc_pixel[0] / 255.0;
                    uc_pixel += 3;
                    ++i;
                }
            }

            // Running inference
            doInference(*context, stream, buffers, data, prob, BATCH_SIZE);
            std::vector<std::vector < Yolo::Detection >> batch_res(BATCH_SIZE);
            auto& res = batch_res[batch];
            nms(res, &prob[batch * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);

            // Displaying 'raw' objects
            for (size_t j = 0; j < res.size(); j++) {

                std::ostringstream label_ss;
                label_ss << res[j].class_id << ": " << std::fixed << std::setprecision(2) << res[j].conf;
                auto label = label_ss.str();

                if(std::stoi(label.c_str()) == 0)
                {
                    cv::Rect r = get_rect(left_cv_rgb, res[j].bbox);
                    cv::rectangle(left_cv_rgb, r, cv::Scalar(0x27, 0xC1, 0x36), 3);
                    int baseline;
                    auto label_bg_sz = cv::getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
                    cv::rectangle(left_cv_rgb, cv::Point(r.x, r.y - label_bg_sz.height - baseline - 10), cv::Point(r.x + 50 + label_bg_sz.width, r.y+20), (255,0,255), cv::FILLED);

                    cv::putText(left_cv_rgb, label.c_str(), cv::Point(r.x, r.y - baseline - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0xFF, 0xFF, 0xFF));

                    float arr[4], cluster_arr[4];
                    int ch;

                    // mid-point of bounding box of detected object
                    draw_mid_points(left_cv_rgb, r, point_cloud, baseline, arr, ch = 0);

                    

                    // Write object distance text at top left og bounding box
                    dist = std::to_string(arr[3]/1000);
                    dist = "D : " + dist;
                    cv::putText(left_cv_rgb, dist, cv::Point(r.x, r.y + baseline + 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0xFF, 0xFF, 0xFF));

                    float bb_mid_point_y = arr[0];
                    float d_left, d_right, object_distance;

                    object_distance = arr[3];
                    std::cout << label << " : " << object_distance/1000 << std::endl;
                    
                    float bed_mid_pcx, bed_mid_pcy, bed_mid_pcz; 
                    bed_mid_pcx = arr[0];
                    bed_mid_pcy = arr[1];
                    bed_mid_pcz = arr[2];

                    myfile.open (argv[3]);

                    int I_x = left_sl.getWidth() / 2;
                    int I_y = left_sl.getHeight() / 2;

                    cv::circle(left_cv_rgb, cv::Point(I_x , I_y), 8, (0, 255, 0), cv::FILLED);

                    sl::float4 pcl_img;
                    point_cloud.getValue(I_x, I_y, &pcl_img);




                    if(std::isfinite(pcl_img.z))
                    {
                        float C_I_dis = sqrt(pcl_img.x * pcl_img.x + pcl_img.y * pcl_img.y + pcl_img.z * pcl_img.z);
                        std::cout<<"Distance to Camera at {"<<I_x<<";"<<I_y<<"}: "<<C_I_dis<<"mm"<<std::endl;
                        myfile 
                        << pcl_img.x/1000 << "," 
                        << pcl_img.y/1000 << "," 
                        << pcl_img.z/1000 << ",";
                    }
                    else
                    {
                        std::cout<<"The Distance can not be computed at {"<<I_x<<";"<<I_y<<"}"<<std::endl;
                    }

                    // for points cluster around mid point of bounding box
                    float min_x = 0, min_y = 0, min_z = 0;
                    float max = 0, min = 0;

                    draw_points_cluster(left_cv_rgb, r, point_cloud, cluster_arr, ch = 0);

                    min_x = cluster_arr[0];
                    min_y = cluster_arr[1];
                    min_z = cluster_arr[2];
                    min = cluster_arr[3];

                    float C_O_dis = sqrt(min_x*min_x + min_y*min_y + min_z*min_z);
                    std::cout<<"Distance to camera from detected object at {"<<(r.x + r.width)/2<<";"<<(r.y + r.height)/2<<"}: "<<C_O_dis<<"mm"<<std::endl;
                    myfile 
                    << min_x/1000 << "," 
                    << min_y/1000 << "," 
                    << min_z/1000 ;


                    // sl::float4 object_img;
                    // point_cloud.getValue((r.x + r.width)/2, (r.y + r.height)/2, &object_img);

                    // if(std::isfinite(object_img.z))
                    // {
                    //     float C_I_dis = sqrt(object_img.x * object_img.x + object_img.y * object_img.y + object_img.z * object_img.z);
                    //     std::cout<<"Distance to camera from detected object at {"<<(r.x + r.width)/2<<";"<<(r.y + r.height)/2<<"}: "<<C_I_dis<<"mm"<<std::endl;
                    //     myfile 
                    //     << object_img.x/1000 << "," 
                    //     << object_img.y/1000 << "," 
                    //     << object_img.z/1000 ;
                    // }
                    // else
                    // {
                    //     std::cout<<"Distance to camera from detected object can not be computed at {"<<(r.x + r.width)/2<<";"<<(r.y + r.height)/2<<"}"<<std::endl;
                    // }


                    myfile << "\n";
                    myfile.close();
                    // depth_points.clear();
                }

                
            }
            cv::imshow("Objects", left_cv_rgb);
            int key = cv::waitKey(10); 
            if (key == 27 || key == 'q') exit_flag = true;

            frame_count++;
        }
    }

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(buffers[inputIndex]));
    CUDA_CHECK(cudaFree(buffers[outputIndex]));
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();

    return 0;
}
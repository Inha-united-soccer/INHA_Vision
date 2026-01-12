#include "booster_vision/model/trt/impl.h"
#include "booster_vision/model/trt/logging.h"

#include <fstream>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <stdexcept>

// 현재 우리는 10.3 버전을 사용하고 있으므로, 해당 버전에 맞는 코드를 작성해야함 
// 현재 10.3 코드는 전처리와 후처리가 CPU 중심으로 짜여 있으므로, 8.6만큼의 극한의 속도를 내고 싶다면 나중에 전처리를 다시 CUDA 커널 방식으로 교체하는 작업이 필요할 수 있
#if (NV_TENSORRT_MAJOR == 10) && (NV_TENSORRT_MINOR == 3)

namespace {

Logger logger;

void MemcpyBuffers(void* dstPtr, void const* srcPtr, size_t byteSize, cudaMemcpyKind memcpyType, bool const async, cudaStream_t& stream) {
    if (async) {
        cudaMemcpyAsync(dstPtr, srcPtr, byteSize, memcpyType, stream);
    } else {
        cudaMemcpy(dstPtr, srcPtr, byteSize, memcpyType);
    }
}

bool PreProcess(const cv::Mat& img, const long int* model_input_shape, float* input_buff, std::vector<float>& factors) {
    cv::Mat mat;
    int rh = img.rows;
    int rw = img.cols;
    int rc = img.channels();

    cv::cvtColor(img, mat, cv::COLOR_BGR2RGB);
    int max_img_len = std::max(rw, rh);  // 使用std::max以清晰
    const int model_input_width = model_input_shape[0];
    const int model_input_height = model_input_shape[1];
    assert(model_input_width == 640 && "model_input_width must be 640");
    assert(model_input_width == model_input_height && "model must be square input");  // 添加断言确保方形

    float factor = static_cast<float>(max_img_len) / model_input_width;
    factors.emplace_back(factor);
    factors.emplace_back(factor);  // 统一因子，保持纵横比

    // 使用灰色填充（114,114,114）
    cv::Mat max_img(max_img_len, max_img_len, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Rect roi(0, 0, rw, rh);
    mat.copyTo(max_img(roi));  // 拷贝到左上角，填充在底部/右侧

    cv::Mat resized_img;
    cv::resize(max_img, resized_img, cv::Size(model_input_width, model_input_width), 0.0f, 0.0f, cv::INTER_LINEAR);
    resized_img.convertTo(resized_img, CV_32FC3, 1.0 / 255.0);
    rh = resized_img.rows;
    rw = resized_img.cols;
    rc = resized_img.channels();
    
    for (int i = 0; i < rc; ++i) {
        cv::extractChannel(resized_img, cv::Mat(rh, rw, CV_32FC1, input_buff + i * rh * rw), i);
    }
    return true;
}

cv::Rect ProcessBoundingBox(const float* data, const std::vector<float>& factors, 
                           int img_width, int img_height) {
    float x = data[0];
    float y = data[1];
    float w = data[2];
    float h = data[3];

    // 计算初始值（可能负或超出）
    int left = static_cast<int>((x - 0.5f * w) * factors[0]);
    int top = static_cast<int>((y - 0.5f * h) * factors[1]);
    int width = static_cast<int>(w * factors[0]);
    int height = static_cast<int>(h * factors[1]);

    // 裁剪left/top到>=0
    int clipped_left = std::max(0, left);
    int clipped_top = std::max(0, top);

    // 计算独占右/下边界，并裁剪到img_width/img_height
    int clipped_right = std::min(img_width, left + width);  // 独占右边界
    int clipped_bottom = std::min(img_height, top + height);  // 独占下边界

    // 计算新宽度/高度
    int new_width = clipped_right - clipped_left;
    int new_height = clipped_bottom - clipped_top;

    // 检查是否有效
    if (new_width <= 0 || new_height <= 0 || new_width < 3 || new_height < 3) {
        return cv::Rect(0, 0, 0, 0);  // 无效
    }

    return cv::Rect(clipped_left, clipped_top, new_width, new_height);
}
}

void YoloV8DetectorTRT::Init(std::string model_path) {
    if (model_path.find(".engine") == std::string::npos) {
        throw std::runtime_error("incorrect model name: " + model_path);
    }
    
    if (!LoadEngine()) {
        throw std::runtime_error("Failed to load engine from " + model_path);
    }

    input_size_ = model_input_dims_.d[0] * model_input_dims_.d[1] * model_input_dims_.d[2] * model_input_dims_.d[3];
    output_size_ = model_output_dims_.d[0] * model_output_dims_.d[1] * model_output_dims_.d[2];
    input_buff_ = (float*)malloc(input_size_ * sizeof(float));
    output_buff_ = (float*)malloc(output_size_ * sizeof(float));
    cudaMalloc(&input_mem_, input_size_ * sizeof(float));
    cudaMalloc(&output_mem_, output_size_ * sizeof(float));
    if (async_infer_)
    {
        cudaStreamCreate(&stream_);
    }
    else
    {
        bindings_.emplace_back(input_mem_);
        bindings_.emplace_back(output_mem_);
    }

    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
    if (!context_)
    {
        // return false;
        throw std::runtime_error("Failed to create execution context");
    }


    context_->setTensorAddress(engine_->getIOTensorName(0), input_mem_);
    context_->setTensorAddress(engine_->getIOTensorName(engine_->getNbIOTensors() - 1), output_mem_);
  
    std::cout << "det model initialization, done!"  << std::endl;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorTRT::Inference(const cv::Mat& img) {
    std::vector<float> factors;
    if (!PreProcess(img, model_input_dims_.d + 2, input_buff_, factors))
    {
        return {};
    }

    // Memcpy from host input buffers to device input buffers
    MemcpyBuffers(input_mem_,input_buff_, input_size_ * sizeof(float),cudaMemcpyHostToDevice, async_infer_, stream_);

    bool status = false;
    if (async_infer_)
    {
        status = context_->enqueueV3(stream_);
    }
    else
    {
        status = context_->executeV2(bindings_.data());
    }
    
    if (!status)
    {
        return {};
    }

    // Memcpy from device output buffers to host output buffers
    MemcpyBuffers(output_buff_, output_mem_, output_size_ * sizeof(float), cudaMemcpyDeviceToHost,async_infer_, stream_);

    if (async_infer_)
    {
        cudaStreamSynchronize(stream_);
    }

    img_width_ = img.cols;
    img_height_ = img.rows;
    std::vector<booster_vision::DetectionRes> outputs = PostProcess(factors);

    std::cout << "finish inference " << std::endl;
    return outputs;
}

YoloV8DetectorTRT::~YoloV8DetectorTRT() {
  cudaStreamDestroy(stream_);
  cudaFree(input_mem_);
  cudaFree(output_mem_);
  free(input_buff_);
  free(output_buff_);
}


bool YoloV8DetectorTRT::LoadEngine()
{
    std::ifstream input(model_path_, std::ios::binary);
    if (!input)
    {
        return false;
    }
    input.seekg(0, input.end);
    const size_t fsize = input.tellg();
    input.seekg(0, input.beg);
    std::vector<char> bytes(fsize);
    input.read(bytes.data(), fsize);

    runtime_ = std::shared_ptr<nvinfer1::IRuntime>(createInferRuntime(logger));
    engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(bytes.data(), bytes.size()), InferDeleter());
    if (!engine_)
        return false;
    
    int nbio = engine_->getNbIOTensors();
    const char* inputname = engine_->getIOTensorName(0);
    const char* outputname = engine_->getIOTensorName(engine_->getNbIOTensors() - 1);
    Dims input_shape = engine_->getTensorShape(inputname);
    Dims output_shape = engine_->getTensorShape(outputname);
    model_input_dims_ = Dims4(input_shape.d[0], input_shape.d[1], input_shape.d[2], input_shape.d[3]);
    model_output_dims_ = Dims4(output_shape.d[0], output_shape.d[1], output_shape.d[2], output_shape.d[3]);
    std::cout << "model input dims: " << input_shape.d[0] << " " << input_shape.d[1] << " " << input_shape.d[2] << " " << input_shape.d[3] << std::endl;
    std::cout << "model output dims: " << output_shape.d[0] << " " << output_shape.d[1] << " " << output_shape.d[2] << std::endl;
   

    return true;
}


std::vector<booster_vision::DetectionRes> YoloV8DetectorTRT::PostProcess(std::vector<float> factors)
{
    const int outputSize = model_output_dims_.d[1];
    //float* output = static_cast<float*>(output_buff_);
    cv::Mat outputs(outputSize, 8400, CV_32F, output_buff_);

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    // Preprocessing output results
    const int class_num = outputSize - 4; // 4 for box[x,y,w,h]
    int rows = outputs.size[0];
    int dimensions = outputs.size[1];
    bool yolov8 = false;

    if (dimensions > rows)
    {
        yolov8 = true;
        rows = outputs.size[1];
        dimensions = outputs.size[0];

        outputs = outputs.reshape(1, dimensions);
        cv::transpose(outputs, outputs);
    }

    float* data = (float*)outputs.data;
    for (int i = 0; i < rows; ++i)
    {
        float* classes_scores = data + 4;

        cv::Mat scores(1, class_num, CV_32FC1, classes_scores);
        // std::cout << "scores: " << scores << std::endl;
        cv::Point class_id;
        double max_class_score;

        minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

        if (max_class_score > confidence_)
        {
            auto bbox = ProcessBoundingBox(data, factors, img_width_, img_height_);
            if (bbox.width <= 0 || bbox.height <= 0) continue;

            boxes.push_back(bbox);
            confidences.push_back(max_class_score);
            class_ids.push_back(class_id.x);
        }

        data += dimensions;
    }
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, 0.25, 0.4, nms_result);

    std::vector<booster_vision::DetectionRes> detections{};
    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];

        booster_vision::DetectionRes result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.bbox = boxes[idx];

        detections.push_back(result);
    }

    return detections;
}

bool YoloV8SegmentorTRT::LoadEngine() {
    std::ifstream input(model_path_, std::ios::binary);
    if (!input)
    {
        return false;
    }
    input.seekg(0, input.end);
    const size_t fsize = input.tellg();
    input.seekg(0, input.beg);
    std::vector<char> bytes(fsize);
    input.read(bytes.data(), fsize);

    runtime_ = std::shared_ptr<nvinfer1::IRuntime>(createInferRuntime(logger));
    engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(bytes.data(), bytes.size()), InferDeleter());
    if (!engine_)
        return false;
    
    int nb_io = engine_->getNbIOTensors();
    const char* input_name = engine_->getIOTensorName(0);
    const char* output1_name = engine_->getIOTensorName(nb_io - 2);
    const char* output2_name = engine_->getIOTensorName(nb_io - 1);
    Dims input_shape = engine_->getTensorShape(input_name);
    Dims output1_shape = engine_->getTensorShape(output1_name);
    Dims output2_shape = engine_->getTensorShape(output2_name);
    model_input_dims_ = Dims4(input_shape.d[0], input_shape.d[1], input_shape.d[2], input_shape.d[3]);
    model_output1_dims_ = Dims4(output1_shape.d[0], output1_shape.d[1], output1_shape.d[2], output1_shape.d[3]);
    model_output2_dims_ = Dims4(output2_shape.d[0], output2_shape.d[1], output2_shape.d[2], output2_shape.d[3]);
    std::cout << "model input dims: " << input_shape.d[0] << " " << input_shape.d[1] << " " << input_shape.d[2] << " " << input_shape.d[3] << std::endl;
    std::cout << "model detection output dims: " << output1_shape.d[0] << " " << output1_shape.d[1] << " " << output1_shape.d[2] << std::endl;
    std::cout << "model mask output dims: " << output2_shape.d[0] << " " << output2_shape.d[1] << " " << output2_shape.d[2] << std::endl;

    return true;
}

void YoloV8SegmentorTRT::Init(std::string model_path) {
    if (model_path.find(".engine") == std::string::npos) {
        throw std::runtime_error("incorrect model name: " + model_path);
    }
    
    if (!LoadEngine()) {
        throw std::runtime_error("Failed to load engine from " + model_path);
    }

    input_size_ = model_input_dims_.d[0] * model_input_dims_.d[1] * model_input_dims_.d[2] * model_input_dims_.d[3];
    det_output_size_ = model_output1_dims_.d[0] * model_output1_dims_.d[1] * model_output1_dims_.d[2];
    mask_output_size_ = model_output2_dims_.d[0] * model_output2_dims_.d[1] * model_output2_dims_.d[2] * model_output2_dims_.d[3];
    input_buff_ = (float*)malloc(input_size_ * sizeof(float));
    det_output_buff_ = (float*)malloc(det_output_size_ * sizeof(float));
    mask_output_buff_ = (float*)malloc(mask_output_size_ * sizeof(float));

    cudaMalloc(&input_mem_, input_size_ * sizeof(float));
    cudaMalloc(&det_output_mem_, det_output_size_ * sizeof(float));
    cudaMalloc(&mask_output_mem_, mask_output_size_ * sizeof(float));
    if (async_infer_)
    {
        cudaStreamCreate(&stream_);
    }
    else
    {
        bindings_.emplace_back(input_mem_);
        bindings_.emplace_back(det_output_mem_);
        bindings_.emplace_back(mask_output_mem_);
    }

    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
    if (!context_)
    {
        // return false;
        throw std::runtime_error("Failed to create execution context");
    }


    context_->setTensorAddress(engine_->getIOTensorName(0), input_mem_);
    context_->setTensorAddress(engine_->getIOTensorName(engine_->getNbIOTensors() - 2), det_output_mem_);
    context_->setTensorAddress(engine_->getIOTensorName(engine_->getNbIOTensors() - 1), mask_output_mem_);
  
    std::cout << "seg model initialization, done!"  << std::endl;
}

std::vector<booster_vision::SegmentationRes> YoloV8SegmentorTRT::Inference(const cv::Mat& img) {
    std::vector<float> factors;
    if (!PreProcess(img, model_input_dims_.d + 2, input_buff_, factors))
    {
        return {};
    }

    // Memcpy from host input buffers to device input buffers
    MemcpyBuffers(input_mem_,input_buff_, input_size_ * sizeof(float),cudaMemcpyHostToDevice, async_infer_, stream_);

    bool status = false;
    if (async_infer_)
    {
        status = context_->enqueueV3(stream_);
    }
    else
    {
        status = context_->executeV2(bindings_.data());
    }
    
    if (!status)
    {
        return {};
    }

    // Memcpy from device output buffers to host output buffers
    MemcpyBuffers(det_output_buff_, det_output_mem_, det_output_size_ * sizeof(float), cudaMemcpyDeviceToHost, async_infer_, stream_);
    MemcpyBuffers(mask_output_buff_, mask_output_mem_, mask_output_size_ * sizeof(float), cudaMemcpyDeviceToHost, async_infer_, stream_);

    if (async_infer_)
    {
        cudaStreamSynchronize(stream_);
    }

    img_width_ = img.cols;
    img_height_ = img.rows;
    std::vector<booster_vision::SegmentationRes> outputs = PostProcess(factors);

    std::cout << "finish inference " << std::endl;
    return outputs;
}

std::vector<booster_vision::SegmentationRes> YoloV8SegmentorTRT::PostProcess(std::vector<float> factors) {
    cv::Mat det_outputs(model_output1_dims_.d[1], model_output1_dims_.d[2], CV_32F, det_output_buff_);
    std::vector<cv::Mat> masks;
    const int mask_num = model_output2_dims_.d[1];
    const int mask_height = model_output2_dims_.d[2];
    const int mask_width = model_output2_dims_.d[3];
    for (int i = 0; i < mask_num; ++i) {
        cv::Mat mask(mask_height, mask_width, CV_32F, mask_output_buff_ + i * mask_height * mask_width);
        masks.push_back(mask);
    }

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<float>> masks_scores;
    // Preprocessing output results
    const int class_num = model_output1_dims_.d[1] - 4 - 32; // 4 for box[x,y,w,h], 32 for mask number
    int rows = det_outputs.size[0];
    int dimensions = det_outputs.size[1];
    bool yolov8 = false;

    if (dimensions > rows)
    {
        yolov8 = true;
        rows = det_outputs.size[1];
        dimensions = det_outputs.size[0];

        det_outputs = det_outputs.reshape(1, dimensions);
        cv::transpose(det_outputs, det_outputs);
    }

    float* data = (float*)det_outputs.data;
    for (int i = 0; i < rows; ++i)
    {
        float* classes_scores = data + 4;

        cv::Mat scores(1, class_num, CV_32FC1, classes_scores);
        // std::cout << "scores: " << scores << std::endl;
        cv::Point class_id;
        double max_class_score;

        minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

        if (max_class_score > confidence_)
        {
            auto bbox = ProcessBoundingBox(data, factors, img_width_, img_height_);
            if (bbox.width <= 0 || bbox.height <= 0) continue;

            boxes.push_back(bbox);
            confidences.push_back(max_class_score);
            class_ids.push_back(class_id.x);

            std::vector<float> mask_scores;
            float* scores = data + 4 + class_num; // Skip the box and class scores
            for (int j = 0; j < mask_num; ++j) {
                mask_scores.push_back(scores[j]);
            }
            masks_scores.push_back(mask_scores);
        }

        data += dimensions;
    }
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, 0.25, 0.4, nms_result);

    std::vector<booster_vision::SegmentationRes> segmentations{};
    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];

        booster_vision::SegmentationRes result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.bbox = boxes[idx];
        
        float mask_resize_factor = mask_width / 1280.0;
        cv::Rect resized_bbox(
            int(result.bbox.x * mask_resize_factor),
            int(result.bbox.y * mask_resize_factor),
            int(result.bbox.width * mask_resize_factor),
            int(result.bbox.height * mask_resize_factor)
        );

        // reconstruct mask
        auto mask_score = masks_scores[idx];
        cv::Mat mask = cv::Mat::zeros(mask_height, mask_width, CV_32F);
        for (int v = resized_bbox.y; v < resized_bbox.y + resized_bbox.height; ++v) {
            for (int u = resized_bbox.x; u < resized_bbox.x + resized_bbox.width; ++u) {
                float score = 0.0f;
                for (int j = 0; j < mask_num; ++j) {
                    score += masks[j].at<float>(v, u) * mask_score[j];
                }
                score =  1.0 / (1.0 + exp(-score)); // Sigmoid activation
                mask.at<float>(v, u) = score;
            }
        }
        cv::Rect mask_bbox(0, 0, int(img_width_ * mask_resize_factor), int(img_height_  * mask_resize_factor));
        cv::resize(mask(mask_bbox), result.mask, cv::Size(img_width_, img_height_), 0, 0, cv::INTER_LINEAR);
        result.mask = result.mask > 0.5; // Binarize the mask
        result.mask.convertTo(result.mask, CV_8U, 255.0); // Convert to 8-bit mask

        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(result.mask, result.contour, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);
        segmentations.push_back(result);
    }

    return segmentations;
}

YoloV8SegmentorTRT::~YoloV8SegmentorTRT() {
  cudaStreamDestroy(stream_);
  cudaFree(input_mem_);
  cudaFree(det_output_mem_);
  cudaFree(mask_output_mem_);
  free(input_buff_);
  free(det_output_buff_);
  free(mask_output_buff_);
}

#endif
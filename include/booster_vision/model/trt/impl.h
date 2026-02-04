#pragma once
#include <string>
#include <NvInfer.h>
#include <random>
#include "booster_vision/model/detector.h"

using namespace nvinfer1;

#if (NV_TENSORRT_MAJOR == 10) && (NV_TENSORRT_MINOR == 3)

struct InferDeleter
{
	template <typename T>
	void operator()(T* obj) const
	{
		delete obj;
	}
};

class YoloV8DetectorTRT : public booster_vision::YoloV8Detector {
 public:
  YoloV8DetectorTRT(const std::string& path) : booster_vision::YoloV8Detector(path) {
    Init(path);
  }
  ~YoloV8DetectorTRT();

  void Init(std::string model_path) override;
  std::vector<booster_vision::DetectionRes> Inference(const cv::Mat& img) override;

 private:
	bool LoadEngine();

	std::vector<booster_vision::DetectionRes> PostProcess(std::vector<float> factors);

	std::shared_ptr<nvinfer1::IRuntime> runtime_;
	std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IExecutionContext> context_;
	cudaStream_t stream_ = 0;
	
	nvinfer1::Dims model_input_dims_;
	nvinfer1::Dims model_output_dims_;

  bool async_infer_ = true;
	size_t input_size_;
	size_t output_size_;
  int img_width_ = 0;
  int img_height_ = 0;
	std::vector<void*> bindings_;
	void* input_mem_{ nullptr };
	void* output_mem_{ nullptr };

	float* input_buff_;
	float* output_buff_;

	bool squre_input_ = true;
};

#endif
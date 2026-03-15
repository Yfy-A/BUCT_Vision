#ifndef RM_VISION_CPP__TRT_YOLO_HPP_
#define RM_VISION_CPP__TRT_YOLO_HPP_

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace rm_vision_cpp
{

// TensorRT 日志回调
class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override;
};

struct Detection
{
  float x1, y1, x2, y2;  // 边界框左上右下坐标
  float confidence;       // 置信度
  int class_id;           // 类别 ID
};

class TrtYolo
{
public:
  TrtYolo(const std::string & engine_path, float conf_thresh = 0.25f, float nms_thresh = 0.45f);
  ~TrtYolo();

  // 禁止拷贝
  TrtYolo(const TrtYolo &) = delete;
  TrtYolo & operator=(const TrtYolo &) = delete;

  // 执行推理，返回检测结果列表
  std::vector<Detection> detect(const cv::Mat & image);

  // 获取模型输入尺寸
  int input_width() const { return input_w_; }
  int input_height() const { return input_h_; }

private:
  enum class OutputLayout
  {
    kChannelsFirst,   // [4+num_classes, num_detections]
    kDetectionsFirst, // [num_detections, 4+num_classes]
    kEndToEndN6       // [num_detections, 6] => [x1, y1, x2, y2, score, class_id]
  };

  void load_engine(const std::string & engine_path);
  void allocate_buffers();
  void preprocess(const cv::Mat & image, float * input_blob);
  std::vector<Detection> postprocess(float * output, int num_detections, int num_classes,
    float scale_x, float scale_y, float pad_x, float pad_y, int image_w, int image_h);
  void nms(std::vector<Detection> & detections);

  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  // GPU 缓冲区 (TensorRT 10.x API)
  void * input_buffer_{nullptr};
  void * output_buffer_{nullptr};
  std::string input_name_;
  std::string output_name_;

  int input_h_{640};
  int input_w_{640};
  int output_dim1_{0};
  int output_dim2_{0};
  int output_num_detections_{0};
  int output_num_classes_{0};
  OutputLayout output_layout_{OutputLayout::kChannelsFirst};

  float conf_thresh_;
  float nms_thresh_;

  // 复用 CPU 侧缓冲，避免每帧分配释放
  std::vector<float> host_input_;
  std::vector<float> host_output_;

  cudaStream_t stream_{nullptr};
};

}  // namespace rm_vision_cpp

#endif  // RM_VISION_CPP__TRT_YOLO_HPP_

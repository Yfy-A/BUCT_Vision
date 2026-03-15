#include "rm_vision_cpp/trt_yolo.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>

namespace rm_vision_cpp
{

void TrtLogger::log(Severity severity, const char * msg) noexcept
{
  if (severity <= Severity::kWARNING) {
    std::cerr << "[TensorRT] " << msg << std::endl;
  }
}

TrtYolo::TrtYolo(const std::string & engine_path, float conf_thresh, float nms_thresh)
: conf_thresh_(conf_thresh), nms_thresh_(nms_thresh)
{
  cudaStreamCreate(&stream_);
  load_engine(engine_path);
  allocate_buffers();
}

TrtYolo::~TrtYolo()
{
  if (input_buffer_) cudaFree(input_buffer_);
  if (output_buffer_) cudaFree(output_buffer_);
  if (stream_) cudaStreamDestroy(stream_);
}

void TrtYolo::load_engine(const std::string & engine_path)
{
  std::ifstream file(engine_path, std::ios::binary);
  if (!file.good()) {
    throw std::runtime_error("Failed to open engine file: " + engine_path);
  }

  file.seekg(0, std::ios::end);
  size_t size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<char> engine_data(size);
  file.read(engine_data.data(), size);

  // Ultralytics 导出的 .engine 文件带有自定义头部:
  // [4 bytes header_size (uint32 LE)] [header_size bytes JSON 元数据] [TRT plan]
  // 检测并跳过该头部，直接传递 TRT plan 给 deserializeCudaEngine
  const char * plan_data = engine_data.data();
  size_t plan_size = size;
  if (size > 4) {
    uint32_t header_size = 0;
    std::memcpy(&header_size, engine_data.data(), sizeof(uint32_t));
    size_t offset = sizeof(uint32_t) + header_size;
    if (header_size > 0 && header_size < size && offset < size &&
        engine_data[4] == '{')
    {
      std::cerr << "[TrtYolo] Detected Ultralytics engine header ("
                << header_size << " bytes), skipping to TRT plan." << std::endl;
      plan_data = engine_data.data() + offset;
      plan_size = size - offset;
    }
  }

  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    throw std::runtime_error("Failed to create TensorRT runtime");
  }

  engine_.reset(runtime_->deserializeCudaEngine(plan_data, plan_size));
  if (!engine_) {
    throw std::runtime_error("Failed to deserialize CUDA engine");
  }

  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("Failed to create execution context");
  }
}

void TrtYolo::allocate_buffers()
{
  // TensorRT 10.x API: 使用 getNbIOTensors / getIOTensorName / getTensorShape
  int nb_io = engine_->getNbIOTensors();
  for (int i = 0; i < nb_io; ++i) {
    const char * name = engine_->getIOTensorName(i);
    auto mode = engine_->getTensorIOMode(name);
    auto dims = engine_->getTensorShape(name);

    if (mode == nvinfer1::TensorIOMode::kINPUT) {
      input_name_ = name;
      // 输入维度: [batch, channels, height, width]
      input_h_ = dims.d[2];
      input_w_ = dims.d[3];

      size_t input_size = 1 * 3 * input_h_ * input_w_ * sizeof(float);
      cudaMalloc(&input_buffer_, input_size);
    } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
      output_name_ = name;
      // 常见输出维度:
      // 1) [batch, 4+num_classes, num_detections]
      // 2) [batch, num_detections, 4+num_classes]
      output_dim1_ = dims.d[1];
      output_dim2_ = dims.d[2];

      if (output_dim1_ <= 0 || output_dim2_ <= 0) {
        throw std::runtime_error("Invalid output tensor shape");
      }

      // 自动识别布局，避免把 [N, C] 误解析成 [C, N]
      if (output_dim2_ == 6 || output_dim2_ == 7) {
        output_layout_ = OutputLayout::kEndToEndN6;
        output_num_detections_ = output_dim1_;
        output_num_classes_ = 0;
      } else if (output_dim1_ > output_dim2_) {
        output_layout_ = OutputLayout::kDetectionsFirst;
        output_num_detections_ = output_dim1_;
        output_num_classes_ = output_dim2_ - 4;
      } else {
        output_layout_ = OutputLayout::kChannelsFirst;
        output_num_detections_ = output_dim2_;
        output_num_classes_ = output_dim1_ - 4;
      }

      std::cerr << "[TrtYolo] Output tensor: " << output_name_
                << " shape=[1," << output_dim1_ << "," << output_dim2_ << "]"
                << ", layout="
                << (output_layout_ == OutputLayout::kChannelsFirst ? "[C,N]" :
                  (output_layout_ == OutputLayout::kDetectionsFirst ? "[N,C]" : "[N,6]"))
                << ", num_detections=" << output_num_detections_
                << ", num_classes=" << output_num_classes_ << std::endl;

      if (output_layout_ != OutputLayout::kEndToEndN6 && output_num_classes_ <= 0) {
        throw std::runtime_error("Invalid class count inferred from output tensor");
      }

      size_t output_size = 1 * output_dim1_ * output_dim2_ * sizeof(float);
      cudaMalloc(&output_buffer_, output_size);
    }
  }

  if (input_name_.empty() || output_name_.empty()) {
    throw std::runtime_error("Failed to find input/output tensors in engine");
  }

  host_input_.resize(static_cast<size_t>(3) * input_h_ * input_w_);
  host_output_.resize(static_cast<size_t>(output_dim1_) * output_dim2_);
}

void TrtYolo::preprocess(const cv::Mat & image, float * input_blob)
{
  // letterbox resize: 保持宽高比缩放到模型输入尺寸
  int img_w = image.cols;
  int img_h = image.rows;
  float scale = std::min(
    static_cast<float>(input_w_) / img_w,
    static_cast<float>(input_h_) / img_h);
  int new_w = static_cast<int>(img_w * scale);
  int new_h = static_cast<int>(img_h * scale);

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(new_w, new_h));

  // 创建灰色填充图
  cv::Mat padded(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
  int pad_x = (input_w_ - new_w) / 2;
  int pad_y = (input_h_ - new_h) / 2;
  resized.copyTo(padded(cv::Rect(pad_x, pad_y, new_w, new_h)));

  // BGR -> RGB, HWC -> CHW, 归一化到 [0, 1]
  int area = input_h_ * input_w_;
  for (int row = 0; row < input_h_; ++row) {
    for (int col = 0; col < input_w_; ++col) {
      const auto & pixel = padded.at<cv::Vec3b>(row, col);
      input_blob[0 * area + row * input_w_ + col] = pixel[2] / 255.0f;  // R
      input_blob[1 * area + row * input_w_ + col] = pixel[1] / 255.0f;  // G
      input_blob[2 * area + row * input_w_ + col] = pixel[0] / 255.0f;  // B
    }
  }
}

std::vector<Detection> TrtYolo::detect(const cv::Mat & image)
{
  int img_w = image.cols;
  int img_h = image.rows;
  float scale = std::min(
    static_cast<float>(input_w_) / img_w,
    static_cast<float>(input_h_) / img_h);
  float pad_x = (input_w_ - img_w * scale) / 2.0f;
  float pad_y = (input_h_ - img_h * scale) / 2.0f;

  // 1. 预处理
  size_t input_size = 3 * input_h_ * input_w_;
  preprocess(image, host_input_.data());

  // 2. 拷贝数据到 GPU
  cudaMemcpyAsync(input_buffer_, host_input_.data(),
    input_size * sizeof(float), cudaMemcpyHostToDevice, stream_);

  // 3. 设置 tensor 地址并推理 (TensorRT 10.x API)
  context_->setTensorAddress(input_name_.c_str(), input_buffer_);
  context_->setTensorAddress(output_name_.c_str(), output_buffer_);
  context_->enqueueV3(stream_);
  cudaStreamSynchronize(stream_);

  // 4. 拷贝输出回 CPU
  size_t output_size = output_dim1_ * output_dim2_;
  cudaMemcpy(host_output_.data(), output_buffer_,
    output_size * sizeof(float), cudaMemcpyDeviceToHost);

  // 5. 后处理
  auto detections = postprocess(
    host_output_.data(), output_num_detections_, output_num_classes_,
    scale, scale, pad_x, pad_y, img_w, img_h);

  return detections;
}

std::vector<Detection> TrtYolo::postprocess(
  float * output, int num_detections, int num_classes,
  float scale_x, float scale_y, float pad_x, float pad_y, int image_w, int image_h)
{
  // YOLOv8 输出: [4+num_classes, num_detections]
  // 前4行: cx, cy, w, h (在输入图尺度)
  // 后面: 各类别的置信度
  std::vector<Detection> detections;

  if (output_layout_ == OutputLayout::kEndToEndN6) {
    for (int i = 0; i < num_detections; ++i) {
      const int base = i * output_dim2_;
      float x1 = output[base + 0];
      float y1 = output[base + 1];
      float x2 = output[base + 2];
      float y2 = output[base + 3];
      float conf = output[base + 4];
      int class_id = static_cast<int>(std::round(output[base + 5]));

      if (!std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(x2) || !std::isfinite(y2) ||
        !std::isfinite(conf))
      {
        continue;
      }
      if (conf < conf_thresh_) {
        continue;
      }

      // End2End 引擎常见输出是输入张量(含letterbox)坐标，需要逆变换回原图坐标。
      // 若输出是归一化坐标，先恢复到输入张量尺度。
      if (x2 <= 2.0f && y2 <= 2.0f) {
        x1 *= static_cast<float>(input_w_);
        x2 *= static_cast<float>(input_w_);
        y1 *= static_cast<float>(input_h_);
        y2 *= static_cast<float>(input_h_);
      }

      // 仅当坐标看起来位于模型输入尺度时，执行逆 letterbox。
      const bool looks_like_input_space =
        (x2 <= static_cast<float>(input_w_) + 1.0f) &&
        (y2 <= static_cast<float>(input_h_) + 1.0f);
      if (looks_like_input_space) {
        x1 = (x1 - pad_x) / scale_x;
        x2 = (x2 - pad_x) / scale_x;
        y1 = (y1 - pad_y) / scale_y;
        y2 = (y2 - pad_y) / scale_y;
      }

      x1 = std::clamp(x1, 0.0f, static_cast<float>(image_w - 1));
      y1 = std::clamp(y1, 0.0f, static_cast<float>(image_h - 1));
      x2 = std::clamp(x2, 0.0f, static_cast<float>(image_w - 1));
      y2 = std::clamp(y2, 0.0f, static_cast<float>(image_h - 1));

      if (x2 <= x1 || y2 <= y1) {
        continue;
      }

      detections.push_back(Detection{x1, y1, x2, y2, conf, class_id});
    }
    return detections;
  }

  for (int i = 0; i < num_detections; ++i) {
    // 找到最大类别置信度
    float max_conf = 0.0f;
    int max_class = 0;
    for (int c = 0; c < num_classes; ++c) {
      float conf = 0.0f;
      if (output_layout_ == OutputLayout::kChannelsFirst) {
        conf = output[(4 + c) * num_detections + i];
      } else {
        conf = output[i * (4 + num_classes) + (4 + c)];
      }
      if (conf > max_conf) {
        max_conf = conf;
        max_class = c;
      }
    }

    if (max_conf < conf_thresh_) {
      continue;
    }

    // 获取 cx, cy, w, h (在填充后的输入图尺度)
    float cx = 0.0f;
    float cy = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
    if (output_layout_ == OutputLayout::kChannelsFirst) {
      cx = output[0 * num_detections + i];
      cy = output[1 * num_detections + i];
      w = output[2 * num_detections + i];
      h = output[3 * num_detections + i];
    } else {
      cx = output[i * (4 + num_classes) + 0];
      cy = output[i * (4 + num_classes) + 1];
      w = output[i * (4 + num_classes) + 2];
      h = output[i * (4 + num_classes) + 3];
    }

    if (!(std::isfinite(cx) && std::isfinite(cy) && std::isfinite(w) && std::isfinite(h))) {
      continue;
    }
    if (w <= 1.0f || h <= 1.0f) {
      continue;
    }

    // 转换回原始图像坐标
    float x1 = (cx - w / 2.0f - pad_x) / scale_x;
    float y1 = (cy - h / 2.0f - pad_y) / scale_y;
    float x2 = (cx + w / 2.0f - pad_x) / scale_x;
    float y2 = (cy + h / 2.0f - pad_y) / scale_y;

    // 裁剪到图像边界（逆 letterbox 后坐标可能略越界）
    float max_x = static_cast<float>(image_w);
    float max_y = static_cast<float>(image_h);
    x1 = std::clamp(x1, 0.0f, max_x - 1.0f);
    y1 = std::clamp(y1, 0.0f, max_y - 1.0f);
    x2 = std::clamp(x2, 0.0f, max_x - 1.0f);
    y2 = std::clamp(y2, 0.0f, max_y - 1.0f);

    if (x2 <= x1 || y2 <= y1) {
      continue;
    }

    Detection det;
    det.x1 = x1;
    det.y1 = y1;
    det.x2 = x2;
    det.y2 = y2;
    det.confidence = max_conf;
    det.class_id = max_class;
    detections.push_back(det);
  }

  // NMS
  nms(detections);

  return detections;
}

void TrtYolo::nms(std::vector<Detection> & detections)
{
  // 按置信度降序排序
  std::sort(detections.begin(), detections.end(),
    [](const Detection & a, const Detection & b) {
      return a.confidence > b.confidence;
    });

  std::vector<bool> suppressed(detections.size(), false);
  std::vector<Detection> result;

  for (size_t i = 0; i < detections.size(); ++i) {
    if (suppressed[i]) continue;
    result.push_back(detections[i]);

    for (size_t j = i + 1; j < detections.size(); ++j) {
      if (suppressed[j]) continue;
      if (detections[i].class_id != detections[j].class_id) continue;

      // 计算 IoU
      float ix1 = std::max(detections[i].x1, detections[j].x1);
      float iy1 = std::max(detections[i].y1, detections[j].y1);
      float ix2 = std::min(detections[i].x2, detections[j].x2);
      float iy2 = std::min(detections[i].y2, detections[j].y2);

      float inter = std::max(0.0f, ix2 - ix1) * std::max(0.0f, iy2 - iy1);
      float area_i = (detections[i].x2 - detections[i].x1) * (detections[i].y2 - detections[i].y1);
      float area_j = (detections[j].x2 - detections[j].x1) * (detections[j].y2 - detections[j].y1);
      float iou = inter / (area_i + area_j - inter + 1e-6f);

      if (iou > nms_thresh_) {
        suppressed[j] = true;
      }
    }
  }

  detections = std::move(result);
}

}  // namespace rm_vision_cpp

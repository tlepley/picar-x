// Native Jetson viewer for PI-CAR-X.
// Supported Ultralytics models for this project:
// - YOLOv5 Detect (raw TensorRT outputs, decoded with the YOLOv5/objectness path)
// - YOLOv8 Detect (raw TensorRT outputs, decoded with the Ultralytics Detect path)
// - YOLO11 Detect (raw TensorRT outputs, currently parsed through the same Ultralytics Detect path as YOLOv8)
// - Other Ultralytics Detect exports only if they keep the same axis-aligned box tensor layout
// Not supported in this viewer:
// - Segment, Pose, OBB, Classify task heads
// - Engines exported with a materially different post-NMS/output contract
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <NvInferPlugin.h>
#include <NvInferRuntime.h>
#include <opencv2/core.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

constexpr int kArrowUp = 2490368;
constexpr int kArrowLeft = 2424832;
constexpr int kArrowRight = 2555904;
constexpr int kArrowDown = 2621440;

enum class KeyAction
{
  None,
  Forward,
  Backward,
  Left,
  Right,
  Stop,
  SpeedUp,
  SpeedDown,
  CapturePhoto,
  ToggleVideoRecord,
  StopVideoRecord,
  ToggleDetector,
  Help,
  Quit,
};

enum class ModelTask
{
  Detect,
  Segment,
  Classify,
  Pose,
  Obb,
};

enum class DetectDecoder
{
  Auto,
  YoloV5,
  UltralyticsDetect,
};

struct Detection
{
  int class_id{};
  float confidence{};
  cv::Rect box;
};

struct Track
{
  int track_id{};
  int class_id{};
  float confidence{};
  cv::Rect2f box;
  int hits{};
  int misses{};
  bool confirmed{};
  bool matched_in_frame{};
};

struct LetterboxInfo
{
  float scale{1.0F};
  int pad_x{};
  int pad_y{};
};

struct DetectionPipelineStats
{
  bool detector_ready{false};
  bool detector_enabled{false};
  std::string backend_in_use{"TensorRT"};
  std::string model_name;
  std::string model_task{"detect"};
  std::string decoder{"ultralytics"};
  int detect_every_n_frames{1};
  double last_inference_ms{0.0};
  std::size_t raw_detection_count{0};
  std::size_t display_detection_count{0};
  std::size_t track_count{0};
};

template<typename T>
struct TrtDeleter
{
  void operator()(T * object) const
  {
    delete object;
  }
};

class TensorRtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override;
};

class TerminalKeyboard
{
public:
  TerminalKeyboard() = default;
  ~TerminalKeyboard();

  bool open();
  void close();
  bool isOpen() const;
  std::vector<KeyAction> pollActions();

private:
  bool open_{false};
  int original_flags_{0};
  termios original_termios_{};
};

std::string trim(const std::string & value);
float intersectionOverUnion(const cv::Rect2f & a, const cv::Rect2f & b);
cv::Rect2f blendRect(const cv::Rect2f & current, const cv::Rect & measured, float alpha);
cv::Rect roundedRect(const cv::Rect2f & box, int image_width, int image_height);
std::string toLowerCopy(std::string value);
ModelTask parseModelTask(const std::string & value);
std::string modelTaskName(ModelTask task);
DetectDecoder parseDetectDecoder(const std::string & value);
std::string detectDecoderName(DetectDecoder decoder);
std::vector<std::string> loadClasses(const std::string & path);
std::optional<cv::Mat> decodeImage(const sensor_msgs::msg::Image & msg);
cv::Mat letterbox(const cv::Mat & image, int target_width, int target_height, LetterboxInfo & info);
cv::Rect scaleBoxToImage(
  const cv::Rect2f & box, const LetterboxInfo & info, int image_width, int image_height);
std::size_t elementSize(nvinfer1::DataType type);
std::size_t getVolume(const nvinfer1::Dims & dims);

class TensorRtYoloEngine
{
public:
  void load(const std::string & engine_path, int input_width, int input_height);
  ~TensorRtYoloEngine();

  double lastInferenceMs() const;
  std::string runtimeName() const;
  std::vector<Detection> infer(
    const cv::Mat & frame,
    double score_threshold,
    double confidence_threshold,
    double nms_threshold,
    const std::string & yolo_variant);

private:
  void checkCuda(cudaError_t status, const char * operation);
  void discoverTensors();
  void configureInputShape(int input_width, int input_height);
  void allocateBuffers();
  void fillInputBuffer(const cv::Mat & bgr_image);
  std::vector<Detection> parseDetections(
    const std::vector<float> & output_values,
    const nvinfer1::Dims & output_dims,
    const LetterboxInfo & info,
    int image_width,
    int image_height,
    double score_threshold,
    double confidence_threshold,
    double nms_threshold,
    const std::string & yolo_variant);

  TensorRtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime, TrtDeleter<nvinfer1::IRuntime>> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter<nvinfer1::ICudaEngine>> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter<nvinfer1::IExecutionContext>> context_;
  std::string input_name_;
  std::string output_name_;
  nvinfer1::Dims input_dims_{};
  nvinfer1::Dims output_dims_{};
  nvinfer1::DataType input_dtype_{nvinfer1::DataType::kFLOAT};
  nvinfer1::DataType output_dtype_{nvinfer1::DataType::kFLOAT};
  int input_width_{640};
  int input_height_{640};
  std::size_t input_elements_{0};
  std::size_t output_elements_{0};
  std::size_t input_bytes_{0};
  std::size_t output_bytes_{0};
  std::vector<float> input_host_;
  std::vector<float> output_host_float_;
  std::vector<__half> output_host_half_;
  void * device_input_{nullptr};
  void * device_output_{nullptr};
  cudaStream_t stream_{nullptr};
  double last_inference_ms_{0.0};
};

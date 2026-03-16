#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
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

namespace
{
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
  ToggleDetector,
  Help,
  Quit,
};

struct Detection
{
  int class_id{};
  float confidence{};
  cv::Rect box;
};

struct LetterboxInfo
{
  float scale{1.0F};
  int pad_x{};
  int pad_y{};
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
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity > Severity::kWARNING) {
      return;
    }
    std::cerr << "[TensorRT] " << message << std::endl;
  }
};

class TerminalKeyboard
{
public:
  TerminalKeyboard() = default;

  ~TerminalKeyboard()
  {
    close();
  }

  bool open()
  {
    if (!isatty(STDIN_FILENO)) {
      return false;
    }

    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
      return false;
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      return false;
    }

    const int current_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (current_flags < 0) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      return false;
    }
    original_flags_ = current_flags;
    if (fcntl(STDIN_FILENO, F_SETFL, current_flags | O_NONBLOCK) != 0) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      return false;
    }

    open_ = true;
    return true;
  }

  void close()
  {
    if (!open_) {
      return;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    open_ = false;
  }

  bool is_open() const
  {
    return open_;
  }

  std::vector<KeyAction> poll_actions()
  {
    std::vector<KeyAction> actions;
    if (!open_) {
      return actions;
    }

    unsigned char buffer[32];
    while (true) {
      const ssize_t bytes_read = read(STDIN_FILENO, buffer, sizeof(buffer));
      if (bytes_read <= 0) {
        break;
      }

      for (ssize_t i = 0; i < bytes_read; ++i) {
        const unsigned char c = buffer[i];
        if (c == 0x1B) {
          if (i + 2 < bytes_read && buffer[i + 1] == '[') {
            const unsigned char arrow = buffer[i + 2];
            switch (arrow) {
              case 'A':
                actions.push_back(KeyAction::Forward);
                break;
              case 'B':
                actions.push_back(KeyAction::Backward);
                break;
              case 'C':
                actions.push_back(KeyAction::Right);
                break;
              case 'D':
                actions.push_back(KeyAction::Left);
                break;
              default:
                actions.push_back(KeyAction::Quit);
                break;
            }
            i += 2;
          } else {
            actions.push_back(KeyAction::Quit);
          }
          continue;
        }

        switch (c) {
          case ' ':
            actions.push_back(KeyAction::Stop);
            break;
          case '+':
          case '=':
            actions.push_back(KeyAction::SpeedUp);
            break;
          case '-':
          case '_':
            actions.push_back(KeyAction::SpeedDown);
            break;
          case 'f':
          case 'F':
            actions.push_back(KeyAction::ToggleDetector);
            break;
          case 'h':
          case 'H':
            actions.push_back(KeyAction::Help);
            break;
          case 'x':
          case 'X':
            actions.push_back(KeyAction::Quit);
            break;
          default:
            break;
        }
      }
    }
    return actions;
  }

private:
  bool open_{false};
  int original_flags_{0};
  termios original_termios_{};
};

std::string trim(const std::string & value)
{
  const auto start = value.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(start, end - start + 1);
}

std::vector<std::string> load_classes(const std::string & path)
{
  std::vector<std::string> classes;
  if (path.empty()) {
    return classes;
  }

  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    line = trim(line);
    if (!line.empty()) {
      classes.push_back(line);
    }
  }
  return classes;
}

std::optional<cv::Mat> decode_image(const sensor_msgs::msg::Image & msg)
{
  int channels = 0;
  if (msg.encoding == "mono8") {
    channels = 1;
  } else if (msg.encoding == "rgb8" || msg.encoding == "bgr8") {
    channels = 3;
  } else {
    return std::nullopt;
  }

  const auto expected_size = static_cast<std::size_t>(msg.width) *
    static_cast<std::size_t>(msg.height) * static_cast<std::size_t>(channels);
  if (msg.data.size() != expected_size) {
    return std::nullopt;
  }

  cv::Mat frame(
    static_cast<int>(msg.height),
    static_cast<int>(msg.width),
    channels == 1 ? CV_8UC1 : CV_8UC3,
    const_cast<unsigned char *>(msg.data.data()));
  cv::Mat owned = frame.clone();
  if (msg.encoding == "mono8") {
    cv::cvtColor(owned, owned, cv::COLOR_GRAY2BGR);
  } else if (msg.encoding == "rgb8") {
    cv::cvtColor(owned, owned, cv::COLOR_RGB2BGR);
  }
  return owned;
}

cv::Mat letterbox(const cv::Mat & image, int target_width, int target_height, LetterboxInfo & info)
{
  const float scale = std::min(
    static_cast<float>(target_width) / static_cast<float>(image.cols),
    static_cast<float>(target_height) / static_cast<float>(image.rows));
  const int resized_width = std::max(1, static_cast<int>(std::round(image.cols * scale)));
  const int resized_height = std::max(1, static_cast<int>(std::round(image.rows * scale)));
  const int pad_x = (target_width - resized_width) / 2;
  const int pad_y = (target_height - resized_height) / 2;

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(resized_width, resized_height));

  cv::Mat canvas(target_height, target_width, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect(pad_x, pad_y, resized_width, resized_height)));

  info.scale = scale;
  info.pad_x = pad_x;
  info.pad_y = pad_y;
  return canvas;
}

cv::Rect scale_box_to_image(
  const cv::Rect2f & box, const LetterboxInfo & info, int image_width, int image_height)
{
  const float x = (box.x - static_cast<float>(info.pad_x)) / info.scale;
  const float y = (box.y - static_cast<float>(info.pad_y)) / info.scale;
  const float width = box.width / info.scale;
  const float height = box.height / info.scale;

  const int left = std::max(0, static_cast<int>(std::round(x)));
  const int top = std::max(0, static_cast<int>(std::round(y)));
  const int right = std::min(image_width, static_cast<int>(std::round(x + width)));
  const int bottom = std::min(image_height, static_cast<int>(std::round(y + height)));
  if (right <= left || bottom <= top) {
    return {};
  }
  return cv::Rect(left, top, right - left, bottom - top);
}

std::size_t element_size(nvinfer1::DataType type)
{
  switch (type) {
    case nvinfer1::DataType::kFLOAT:
      return sizeof(float);
    case nvinfer1::DataType::kHALF:
      return sizeof(__half);
    default:
      throw std::runtime_error("Unsupported TensorRT tensor datatype for this app.");
  }
}

std::size_t volume(const nvinfer1::Dims & dims)
{
  std::size_t result = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] < 0) {
      throw std::runtime_error("Dynamic tensor dimensions are not fully specified.");
    }
    result *= static_cast<std::size_t>(dims.d[i]);
  }
  return result;
}

class TensorRtYoloEngine
{
public:
  void load(const std::string & engine_path, int input_width, int input_height)
  {
    initLibNvInferPlugins(&logger_, "");

    std::ifstream stream(engine_path, std::ios::binary);
    if (!stream) {
      throw std::runtime_error("Could not open TensorRT engine file: " + engine_path);
    }
    stream.seekg(0, std::ios::end);
    const auto size = static_cast<std::size_t>(stream.tellg());
    stream.seekg(0, std::ios::beg);

    std::vector<char> bytes(size);
    stream.read(bytes.data(), static_cast<std::streamsize>(size));

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("Failed to create TensorRT runtime.");
    }

    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine_) {
      throw std::runtime_error("Failed to deserialize TensorRT engine.");
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("Failed to create TensorRT execution context.");
    }

    discover_tensors();
    configure_input_shape(input_width, input_height);
    allocate_buffers();
  }

  ~TensorRtYoloEngine()
  {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
    if (device_input_ != nullptr) {
      cudaFree(device_input_);
    }
    if (device_output_ != nullptr) {
      cudaFree(device_output_);
    }
  }

  double last_inference_ms() const
  {
    return last_inference_ms_;
  }

  std::string runtime_name() const
  {
    std::ostringstream out;
    out << "TensorRT";
    if (output_dtype_ == nvinfer1::DataType::kHALF) {
      out << " fp16";
    } else {
      out << " fp32";
    }
    return out.str();
  }

  std::vector<Detection> infer(
    const cv::Mat & frame,
    double score_threshold,
    double confidence_threshold,
    double nms_threshold,
    const std::string & yolo_variant)
  {
    LetterboxInfo info;
    const cv::Mat padded = letterbox(frame, input_width_, input_height_, info);
    fill_input_buffer(padded);

    check_cuda(cudaMemcpyAsync(
      device_input_, input_host_.data(), input_bytes_, cudaMemcpyHostToDevice, stream_),
      "cudaMemcpyAsync input");

    const auto begin = std::chrono::steady_clock::now();
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("TensorRT enqueueV3 failed.");
    }

    if (output_dtype_ == nvinfer1::DataType::kFLOAT) {
      check_cuda(cudaMemcpyAsync(
        output_host_float_.data(), device_output_, output_bytes_, cudaMemcpyDeviceToHost, stream_),
        "cudaMemcpyAsync output");
    } else {
      check_cuda(cudaMemcpyAsync(
        output_host_half_.data(), device_output_, output_bytes_, cudaMemcpyDeviceToHost, stream_),
        "cudaMemcpyAsync output");
    }
    check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
    const auto end = std::chrono::steady_clock::now();
    last_inference_ms_ = std::chrono::duration<double, std::milli>(end - begin).count();

    std::vector<float> output_values(output_elements_);
    if (output_dtype_ == nvinfer1::DataType::kFLOAT) {
      output_values = output_host_float_;
    } else {
      for (std::size_t i = 0; i < output_host_half_.size(); ++i) {
        output_values[i] = __half2float(output_host_half_[i]);
      }
    }

    return parse_detections(
      output_values, output_dims_, info, frame.cols, frame.rows,
      score_threshold, confidence_threshold, nms_threshold, yolo_variant);
  }

private:
  void check_cuda(cudaError_t status, const char * operation)
  {
    if (status != cudaSuccess) {
      throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
    }
  }

  void discover_tensors()
  {
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * name = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT && input_name_.empty()) {
        input_name_ = name;
      } else if (
        engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT &&
        output_name_.empty())
      {
        output_name_ = name;
      }
    }

    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error("TensorRT engine must expose one input and one output tensor.");
    }
  }

  void configure_input_shape(int input_width, int input_height)
  {
    input_width_ = input_width;
    input_height_ = input_height;

    const auto engine_input_dims = engine_->getTensorShape(input_name_.c_str());
    bool has_dynamic_dim = false;
    for (int i = 0; i < engine_input_dims.nbDims; ++i) {
      if (engine_input_dims.d[i] < 0) {
        has_dynamic_dim = true;
        break;
      }
    }

    if (has_dynamic_dim) {
      nvinfer1::Dims4 input_dims{1, 3, input_height_, input_width_};
      if (!context_->setInputShape(input_name_.c_str(), input_dims)) {
        throw std::runtime_error("Failed to set TensorRT dynamic input shape.");
      }
    }

    input_dims_ = context_->getTensorShape(input_name_.c_str());
    output_dims_ = context_->getTensorShape(output_name_.c_str());

    if (input_dims_.nbDims != 4 || input_dims_.d[0] != 1 || input_dims_.d[1] != 3) {
      throw std::runtime_error("This app expects an input tensor shaped like 1x3xHxW.");
    }
  }

  void allocate_buffers()
  {
    input_dtype_ = engine_->getTensorDataType(input_name_.c_str());
    output_dtype_ = engine_->getTensorDataType(output_name_.c_str());
    if (input_dtype_ != nvinfer1::DataType::kFLOAT) {
      throw std::runtime_error("This app currently expects a float TensorRT input tensor.");
    }

    input_elements_ = volume(input_dims_);
    output_elements_ = volume(output_dims_);
    input_bytes_ = input_elements_ * element_size(input_dtype_);
    output_bytes_ = output_elements_ * element_size(output_dtype_);

    input_host_.assign(input_elements_, 0.0F);
    output_host_float_.assign(output_elements_, 0.0F);
    output_host_half_.assign(output_elements_, __float2half(0.0F));

    check_cuda(cudaMalloc(&device_input_, input_bytes_), "cudaMalloc input");
    check_cuda(cudaMalloc(&device_output_, output_bytes_), "cudaMalloc output");
    check_cuda(cudaStreamCreate(&stream_), "cudaStreamCreate");

    if (!context_->setTensorAddress(input_name_.c_str(), device_input_)) {
      throw std::runtime_error("Failed to bind TensorRT input tensor memory.");
    }
    if (!context_->setTensorAddress(output_name_.c_str(), device_output_)) {
      throw std::runtime_error("Failed to bind TensorRT output tensor memory.");
    }
  }

  void fill_input_buffer(const cv::Mat & bgr_image)
  {
    cv::Mat rgb_image;
    cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);

    const int channel_size = input_width_ * input_height_;
    for (int y = 0; y < input_height_; ++y) {
      const auto * row = rgb_image.ptr<cv::Vec3b>(y);
      for (int x = 0; x < input_width_; ++x) {
        const auto pixel = row[x];
        const int index = y * input_width_ + x;
        input_host_[index] = static_cast<float>(pixel[0]) / 255.0F;
        input_host_[channel_size + index] = static_cast<float>(pixel[1]) / 255.0F;
        input_host_[(2 * channel_size) + index] = static_cast<float>(pixel[2]) / 255.0F;
      }
    }
  }

  std::vector<Detection> parse_detections(
    const std::vector<float> & output_values,
    const nvinfer1::Dims & output_dims,
    const LetterboxInfo & info,
    int image_width,
    int image_height,
    double score_threshold,
    double confidence_threshold,
    double nms_threshold,
    const std::string & yolo_variant)
  {
    int rows = 0;
    int cols = 0;
    if (output_dims.nbDims == 3) {
      if (output_dims.d[1] < output_dims.d[2]) {
        rows = output_dims.d[2];
        cols = output_dims.d[1];
      } else {
        rows = output_dims.d[1];
        cols = output_dims.d[2];
      }
    } else if (output_dims.nbDims == 2) {
      rows = output_dims.d[0];
      cols = output_dims.d[1];
    } else {
      throw std::runtime_error("Unsupported TensorRT YOLO output shape.");
    }

    std::vector<float> row_major(static_cast<std::size_t>(rows * cols), 0.0F);
    if (output_dims.nbDims == 3 && output_dims.d[1] < output_dims.d[2]) {
      for (int c = 0; c < cols; ++c) {
        for (int r = 0; r < rows; ++r) {
          row_major[static_cast<std::size_t>(r * cols + c)] =
            output_values[static_cast<std::size_t>(c * rows + r)];
        }
      }
    } else {
      std::copy(output_values.begin(), output_values.begin() + row_major.size(), row_major.begin());
    }

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int row = 0; row < rows; ++row) {
      const float * data = row_major.data() + (row * cols);
      if (cols < 6) {
        continue;
      }

      const float center_x = data[0];
      const float center_y = data[1];
      const float width = data[2];
      const float height = data[3];

      int class_id = -1;
      float confidence = 0.0F;
      if (yolo_variant == "yolov5") {
        const float objectness = data[4];
        if (objectness < static_cast<float>(confidence_threshold)) {
          continue;
        }
        float best_score = 0.0F;
        int best_class = -1;
        for (int i = 5; i < cols; ++i) {
          if (data[i] > best_score) {
            best_score = data[i];
            best_class = i - 5;
          }
        }
        confidence = objectness * best_score;
        class_id = best_class;
      } else {
        float best_score = 0.0F;
        int best_class = -1;
        for (int i = 4; i < cols; ++i) {
          if (data[i] > best_score) {
            best_score = data[i];
            best_class = i - 4;
          }
        }
        confidence = best_score;
        class_id = best_class;
      }

      if (confidence < static_cast<float>(score_threshold) || class_id < 0) {
        continue;
      }

      const cv::Rect scaled = scale_box_to_image(
        cv::Rect2f(center_x - (width / 2.0F), center_y - (height / 2.0F), width, height),
        info, image_width, image_height);
      if (scaled.area() <= 0) {
        continue;
      }

      class_ids.push_back(class_id);
      confidences.push_back(confidence);
      boxes.push_back(scaled);
    }

    std::vector<int> kept_indices;
    cv::dnn::NMSBoxes(
      boxes, confidences, static_cast<float>(score_threshold), static_cast<float>(nms_threshold),
      kept_indices);

    std::vector<Detection> detections;
    detections.reserve(kept_indices.size());
    for (const int index : kept_indices) {
      detections.push_back(Detection{class_ids[index], confidences[index], boxes[index]});
    }
    return detections;
  }

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
}  // namespace

class RemoteJetsonVideoCarNode : public rclcpp::Node
{
public:
  RemoteJetsonVideoCarNode()
  : Node("picarx_remote_yolo_video_car")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/picarx/camera/image_raw");
    speed_topic_ = declare_parameter<std::string>("speed_topic", "/picarx/speed");
    steering_topic_ = declare_parameter<std::string>("steering_topic", "/picarx/steering");
    camera_pan_topic_ = declare_parameter<std::string>("camera_pan_topic", "/picarx/camera_pan");
    camera_tilt_topic_ = declare_parameter<std::string>("camera_tilt_topic", "/picarx/camera_tilt");
    camera_start_service_ = declare_parameter<std::string>(
      "camera_start_service", "/picarx_camera_publisher_node/start");
    camera_stop_service_ = declare_parameter<std::string>(
      "camera_stop_service", "/picarx_camera_publisher_node/stop");
    window_name_ = declare_parameter<std::string>("window_name", "PI-CAR-X Jetson Video Car");
    engine_path_ = declare_parameter<std::string>("engine_path", "");
    classes_path_ = declare_parameter<std::string>("classes_path", "");
    yolo_variant_ = declare_parameter<std::string>("yolo_variant", "yolov8");
    input_width_ = declare_parameter<int>("input_width", 640);
    input_height_ = declare_parameter<int>("input_height", 640);
    detect_every_n_frames_ = std::max(
      1, static_cast<int>(declare_parameter<int>("detect_every_n_frames", 2)));
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.25);
    score_threshold_ = declare_parameter<double>("score_threshold", 0.25);
    nms_threshold_ = declare_parameter<double>("nms_threshold", 0.45);
    base_speed_ = declare_parameter<double>("base_speed", 20.0);
    max_speed_ = declare_parameter<double>("max_speed", 60.0);
    turn_angle_deg_ = declare_parameter<double>("turn_angle_deg", 28.0);
    show_fps_ = declare_parameter<bool>("show_fps", true);
    auto_request_stream_ = declare_parameter<bool>("auto_request_stream", true);

    classes_ = load_classes(classes_path_);

    speed_pub_ = create_publisher<std_msgs::msg::Float32>(speed_topic_, 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float32>(steering_topic_, 10);
    camera_pan_pub_ = create_publisher<std_msgs::msg::Float32>(camera_pan_topic_, 10);
    camera_tilt_pub_ = create_publisher<std_msgs::msg::Float32>(camera_tilt_topic_, 10);

    start_client_ = create_client<std_srvs::srv::Trigger>(camera_start_service_);
    stop_client_ = create_client<std_srvs::srv::Trigger>(camera_stop_service_);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RemoteJetsonVideoCarNode::on_image, this, std::placeholders::_1));

    terminal_keyboard_available_ = terminal_keyboard_.open();
    if (terminal_keyboard_available_) {
      RCLCPP_INFO(
        get_logger(),
        "Terminal keyboard capture enabled. Keep the launch terminal focused for reliable arrow keys.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Terminal keyboard capture unavailable; falling back to OpenCV window key handling.");
    }
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    init_detector();
    center_camera();
  }

  ~RemoteJetsonVideoCarNode() override
  {
    shutdown();
  }

  bool running() const
  {
    return running_;
  }

  void print_help()
  {
    RCLCPP_INFO(
      get_logger(),
      "Controls: arrows in terminal or OpenCV window, space stop, +/- speed, f toggle detector, h help, x/esc quit.");
  }

  void maybe_request_stream_on_start()
  {
    if (auto_request_stream_) {
      request_stream(true);
    }
  }

  void shutdown()
  {
    if (!running_) {
      return;
    }
    running_ = false;
    publish_drive(0.0F, 0.0F);
    request_stream(false);
    terminal_keyboard_.close();
    try {
      cv::destroyWindow(window_name_);
    } catch (const cv::Exception &) {
    }
  }

  void ui_tick()
  {
    cv::Mat frame;
    {
      std::scoped_lock<std::mutex> lock(frame_mutex_);
      if (!latest_frame_.empty()) {
        frame = latest_frame_.clone();
      }
    }

    if (frame.empty()) {
      render_placeholder();
      process_keyboard();
      return;
    }

    ++frame_counter_;
    if (detector_ready_ && detector_enabled_ && (frame_counter_ % detect_every_n_frames_ == 0)) {
      try {
        last_detections_ = detector_.infer(
          frame, score_threshold_, confidence_threshold_, nms_threshold_, yolo_variant_);
        backend_in_use_ = detector_.runtime_name();
        last_inference_ms_ = detector_.last_inference_ms();
      } catch (const std::exception & exc) {
        detector_enabled_ = false;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TensorRT inference failed, detector disabled: %s", exc.what());
      }
    }

    draw_overlay(frame);
    cv::imshow(window_name_, frame);
    process_keyboard();
  }

private:
  void init_detector()
  {
    if (engine_path_.empty()) {
      detector_ready_ = false;
      detector_enabled_ = false;
      RCLCPP_WARN(get_logger(), "YOLO disabled: parameter 'engine_path' is empty.");
      return;
    }

    try {
      detector_.load(engine_path_, input_width_, input_height_);
      detector_ready_ = true;
      detector_enabled_ = true;
      backend_in_use_ = detector_.runtime_name();
      RCLCPP_INFO(get_logger(), "Loaded TensorRT engine: %s", engine_path_.c_str());
    } catch (const std::exception & exc) {
      detector_ready_ = false;
      detector_enabled_ = false;
      RCLCPP_ERROR(get_logger(), "Failed to load TensorRT engine: %s", exc.what());
    }
  }

  void center_camera()
  {
    publish_scalar(camera_pan_pub_, 0.0F);
    publish_scalar(camera_tilt_pub_, 0.0F);
  }

  void request_stream(bool start)
  {
    auto & client = start ? start_client_ : stop_client_;
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(
        get_logger(), "Camera %s service unavailable: %s",
        start ? "start" : "stop", client->get_service_name());
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    const auto result = rclcpp::spin_until_future_complete(
      get_node_base_interface(), future, std::chrono::seconds(5));
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "Camera %s request timed out.", start ? "start" : "stop");
      return;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(
        get_logger(), "Camera %s request failed: %s",
        start ? "start" : "stop", response->message.c_str());
    }
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const auto decoded = decode_image(*msg);
    if (!decoded.has_value()) {
      return;
    }
    std::scoped_lock<std::mutex> lock(frame_mutex_);
    latest_frame_ = decoded.value();
  }

  void render_placeholder()
  {
    cv::Mat placeholder(480, 640, CV_8UC3, cv::Scalar(16, 16, 16));
    cv::putText(
      placeholder, "Waiting for /picarx/camera/image_raw", {24, 80},
      cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(240, 240, 240), 2);
    cv::putText(
      placeholder, "Launch the local hardware stack on the car first.", {24, 120},
      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(180, 180, 180), 1);
    draw_status_text(placeholder);
    cv::imshow(window_name_, placeholder);
  }

  void process_keyboard()
  {
    for (const KeyAction action : terminal_keyboard_.poll_actions()) {
      handle_action(action);
    }

    const int key = cv::waitKeyEx(1);
    if (key < 0) {
      return;
    }

    switch (key) {
      case kArrowUp:
        handle_action(KeyAction::Forward);
        return;
      case kArrowDown:
        handle_action(KeyAction::Backward);
        return;
      case kArrowLeft:
        handle_action(KeyAction::Left);
        return;
      case kArrowRight:
        handle_action(KeyAction::Right);
        return;
      case ' ':
        handle_action(KeyAction::Stop);
        return;
      case '+':
      case '=':
        handle_action(KeyAction::SpeedUp);
        return;
      case '-':
      case '_':
        handle_action(KeyAction::SpeedDown);
        return;
      case 'f':
      case 'F':
        handle_action(KeyAction::ToggleDetector);
        return;
      case 'h':
      case 'H':
        handle_action(KeyAction::Help);
        return;
      case 'x':
      case 'X':
      case 27:
        handle_action(KeyAction::Quit);
        return;
      default:
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Unhandled OpenCV key code: %d", key);
        return;
    }
  }

  void handle_action(KeyAction action)
  {
    switch (action) {
      case KeyAction::Forward:
        publish_drive(
          current_speed_ <= 0.0F ? static_cast<float>(base_speed_) : current_speed_, 0.0F);
        break;
      case KeyAction::Backward:
        publish_drive(
          current_speed_ >= 0.0F ? -static_cast<float>(base_speed_) : current_speed_, 0.0F);
        break;
      case KeyAction::Left:
        publish_drive(static_cast<float>(base_speed_), -static_cast<float>(turn_angle_deg_));
        break;
      case KeyAction::Right:
        publish_drive(static_cast<float>(base_speed_), static_cast<float>(turn_angle_deg_));
        break;
      case KeyAction::Stop:
        publish_drive(0.0F, 0.0F);
        break;
      case KeyAction::SpeedUp:
        current_speed_ = std::min(current_speed_ + 5.0F, static_cast<float>(max_speed_));
        publish_drive(current_speed_, current_steering_);
        break;
      case KeyAction::SpeedDown:
        current_speed_ = std::max(current_speed_ - 5.0F, -static_cast<float>(max_speed_));
        publish_drive(current_speed_, current_steering_);
        break;
      case KeyAction::ToggleDetector:
        detector_enabled_ = detector_ready_ && !detector_enabled_;
        RCLCPP_INFO(get_logger(), "Detector %s", detector_enabled_ ? "enabled" : "disabled");
        break;
      case KeyAction::Help:
        print_help();
        break;
      case KeyAction::Quit:
        shutdown();
        break;
      case KeyAction::None:
        break;
    }
  }

  void publish_scalar(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher,
    float value)
  {
    std_msgs::msg::Float32 msg;
    msg.data = value;
    publisher->publish(msg);
  }

  void publish_drive(float speed, float steering)
  {
    speed = std::clamp(speed, -static_cast<float>(max_speed_), static_cast<float>(max_speed_));
    steering = std::clamp(steering, -30.0F, 30.0F);
    current_speed_ = speed;
    current_steering_ = steering;
    publish_scalar(speed_pub_, current_speed_);
    publish_scalar(steering_pub_, current_steering_);
  }

  std::string class_name(int class_id) const
  {
    if (class_id >= 0 && static_cast<std::size_t>(class_id) < classes_.size()) {
      return classes_[static_cast<std::size_t>(class_id)];
    }
    std::ostringstream out;
    out << "cls_" << class_id;
    return out.str();
  }

  void draw_overlay(cv::Mat & frame)
  {
    for (const auto & detection : last_detections_) {
      cv::rectangle(frame, detection.box, cv::Scalar(0, 220, 255), 2);
      std::ostringstream label;
      label << class_name(detection.class_id) << ' '
            << static_cast<int>(std::round(detection.confidence * 100.0F)) << '%';
      cv::putText(
        frame, label.str(), {detection.box.x, std::max(20, detection.box.y - 8)},
        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 220, 255), 2);
    }

    draw_status_text(frame);
  }

  void draw_status_text(cv::Mat & frame)
  {
    std::ostringstream line1;
    line1 << "speed=" << current_speed_ << " steering=" << current_steering_;
    cv::putText(
      frame, line1.str(), {16, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
      cv::Scalar(255, 255, 255), 2);

    std::ostringstream line2;
    line2 << "detector=" << (detector_enabled_ ? "on" : "off");
    if (detector_ready_) {
      line2 << " backend=" << backend_in_use_;
    }
    line2 << " detections=" << last_detections_.size();
    cv::putText(
      frame, line2.str(), {16, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(200, 200, 200), 2);

    if (show_fps_) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - fps_window_start_).count();
      if (elapsed >= 1.0) {
        last_fps_ = static_cast<double>(fps_frame_counter_) / elapsed;
        fps_window_start_ = now;
        fps_frame_counter_ = 0;
      }
      ++fps_frame_counter_;

      std::ostringstream line3;
      line3 << "fps=" << static_cast<int>(std::round(last_fps_))
            << " infer=" << static_cast<int>(std::round(last_inference_ms_)) << "ms";
      cv::putText(
        frame, line3.str(), {16, 84}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar(200, 200, 200), 2);
    }

    cv::putText(
      frame, "terminal arrows drive | space stop | +/- speed | f detector | x quit",
      {16, frame.rows - 18}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
  }

  std::string image_topic_;
  std::string speed_topic_;
  std::string steering_topic_;
  std::string camera_pan_topic_;
  std::string camera_tilt_topic_;
  std::string camera_start_service_;
  std::string camera_stop_service_;
  std::string window_name_;
  std::string engine_path_;
  std::string classes_path_;
  std::string yolo_variant_;

  int input_width_{640};
  int input_height_{640};
  int detect_every_n_frames_{2};
  double confidence_threshold_{0.25};
  double score_threshold_{0.25};
  double nms_threshold_{0.45};
  double base_speed_{20.0};
  double max_speed_{60.0};
  double turn_angle_deg_{28.0};
  bool show_fps_{true};
  bool auto_request_stream_{true};
  bool detector_ready_{false};
  bool detector_enabled_{false};
  bool running_{true};
  bool terminal_keyboard_available_{false};

  float current_speed_{0.0F};
  float current_steering_{0.0F};
  std::size_t frame_counter_{0};
  double last_inference_ms_{0.0};
  double last_fps_{0.0};
  int fps_frame_counter_{0};
  std::chrono::steady_clock::time_point fps_window_start_{std::chrono::steady_clock::now()};
  std::string backend_in_use_{"TensorRT"};

  TensorRtYoloEngine detector_;
  std::vector<std::string> classes_;
  std::vector<Detection> last_detections_;

  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  TerminalKeyboard terminal_keyboard_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr camera_pan_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr camera_tilt_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RemoteJetsonVideoCarNode>();
  node->print_help();
  node->maybe_request_stream_on_start();

  while (rclcpp::ok() && node->running()) {
    rclcpp::spin_some(node);
    node->ui_tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  node->shutdown();
  rclcpp::shutdown();
  return 0;
}

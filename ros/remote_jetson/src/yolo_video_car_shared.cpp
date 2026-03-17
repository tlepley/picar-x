#include "yolo_video_car_shared.hpp"

void TensorRtLogger::log(Severity severity, const char * message) noexcept
{
  if (severity > Severity::kWARNING) {
    return;
  }
  std::cerr << "[TensorRT] " << message << std::endl;
}

TerminalKeyboard::~TerminalKeyboard()
{
  close();
}

bool TerminalKeyboard::open()
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

void TerminalKeyboard::close()
{
  if (!open_) {
    return;
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
  fcntl(STDIN_FILENO, F_SETFL, original_flags_);
  open_ = false;
}

bool TerminalKeyboard::isOpen() const
{
  return open_;
}

std::vector<KeyAction> TerminalKeyboard::pollActions()
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

std::string trim(const std::string & value)
{
  const auto start = value.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(start, end - start + 1);
}

float intersectionOverUnion(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float left = std::max(a.x, b.x);
  const float top = std::max(a.y, b.y);
  const float right = std::min(a.x + a.width, b.x + b.width);
  const float bottom = std::min(a.y + a.height, b.y + b.height);
  const float width = std::max(0.0F, right - left);
  const float height = std::max(0.0F, bottom - top);
  const float intersection = width * height;
  const float union_area = a.area() + b.area() - intersection;
  if (union_area <= 0.0F) {
    return 0.0F;
  }
  return intersection / union_area;
}

cv::Rect2f blendRect(const cv::Rect2f & current, const cv::Rect & measured, float alpha)
{
  const cv::Rect2f measured_f(
    static_cast<float>(measured.x),
    static_cast<float>(measured.y),
    static_cast<float>(measured.width),
    static_cast<float>(measured.height));
  const float beta = 1.0F - alpha;
  return cv::Rect2f(
    (alpha * current.x) + (beta * measured_f.x),
    (alpha * current.y) + (beta * measured_f.y),
    (alpha * current.width) + (beta * measured_f.width),
    (alpha * current.height) + (beta * measured_f.height));
}

cv::Rect roundedRect(const cv::Rect2f & box, int image_width, int image_height)
{
  const int left = std::clamp(static_cast<int>(std::round(box.x)), 0, image_width);
  const int top = std::clamp(static_cast<int>(std::round(box.y)), 0, image_height);
  const int right = std::clamp(static_cast<int>(std::round(box.x + box.width)), 0, image_width);
  const int bottom = std::clamp(static_cast<int>(std::round(box.y + box.height)), 0, image_height);
  if (right <= left || bottom <= top) {
    return {};
  }
  return cv::Rect(left, top, right - left, bottom - top);
}

std::string toLowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

ModelTask parseModelTask(const std::string & value)
{
  const std::string normalized = toLowerCopy(trim(value));
  if (normalized.empty() || normalized == "detect") {
    return ModelTask::Detect;
  }
  if (normalized == "segment" || normalized == "seg") {
    return ModelTask::Segment;
  }
  if (normalized == "classify" || normalized == "cls") {
    return ModelTask::Classify;
  }
  if (normalized == "pose") {
    return ModelTask::Pose;
  }
  if (normalized == "obb") {
    return ModelTask::Obb;
  }
  throw std::runtime_error("Unsupported model_task: " + value);
}

std::string modelTaskName(ModelTask task)
{
  switch (task) {
    case ModelTask::Detect:
      return "detect";
    case ModelTask::Segment:
      return "segment";
    case ModelTask::Classify:
      return "classify";
    case ModelTask::Pose:
      return "pose";
    case ModelTask::Obb:
      return "obb";
  }
  return "unknown";
}

DetectDecoder parseDetectDecoder(const std::string & value)
{
  const std::string normalized = toLowerCopy(trim(value));
  if (normalized.empty() || normalized == "auto") {
    return DetectDecoder::Auto;
  }
  if (normalized == "yolov5" || normalized == "v5") {
    return DetectDecoder::YoloV5;
  }
  if (
    normalized == "ultralytics" || normalized == "yolov8" || normalized == "yolov9" ||
    normalized == "yolov10" || normalized == "yolo11" || normalized == "yolo26")
  {
    return DetectDecoder::UltralyticsDetect;
  }
  throw std::runtime_error("Unsupported detect_decoder: " + value);
}

std::string detectDecoderName(DetectDecoder decoder)
{
  switch (decoder) {
    case DetectDecoder::Auto:
      return "auto";
    case DetectDecoder::YoloV5:
      return "yolov5";
    case DetectDecoder::UltralyticsDetect:
      return "ultralytics";
  }
  return "unknown";
}

std::vector<std::string> loadClasses(const std::string & path)
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

std::optional<cv::Mat> decodeImage(const sensor_msgs::msg::Image & msg)
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

cv::Rect scaleBoxToImage(
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

std::size_t elementSize(nvinfer1::DataType type)
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

std::size_t getVolume(const nvinfer1::Dims & dims)
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

void TensorRtYoloEngine::load(const std::string & engine_path, int input_width, int input_height)
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

  discoverTensors();
  configureInputShape(input_width, input_height);
  allocateBuffers();
}

TensorRtYoloEngine::~TensorRtYoloEngine()
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

double TensorRtYoloEngine::lastInferenceMs() const
{
  return last_inference_ms_;
}

std::string TensorRtYoloEngine::runtimeName() const
{
  std::ostringstream out;
  out << "TensorRT";
  out << (output_dtype_ == nvinfer1::DataType::kHALF ? " fp16" : " fp32");
  return out.str();
}

std::vector<Detection> TensorRtYoloEngine::infer(
  const cv::Mat & frame,
  double score_threshold,
  double confidence_threshold,
  double nms_threshold,
  const std::string & yolo_variant)
{
  LetterboxInfo info;
  const cv::Mat padded = letterbox(frame, input_width_, input_height_, info);
  fillInputBuffer(padded);

  checkCuda(cudaMemcpyAsync(
    device_input_, input_host_.data(), input_bytes_, cudaMemcpyHostToDevice, stream_),
    "cudaMemcpyAsync input");

  const auto begin = std::chrono::steady_clock::now();
  if (!context_->enqueueV3(stream_)) {
    throw std::runtime_error("TensorRT enqueueV3 failed.");
  }

  if (output_dtype_ == nvinfer1::DataType::kFLOAT) {
    checkCuda(cudaMemcpyAsync(
      output_host_float_.data(), device_output_, output_bytes_, cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpyAsync output");
  } else {
    checkCuda(cudaMemcpyAsync(
      output_host_half_.data(), device_output_, output_bytes_, cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpyAsync output");
  }
  checkCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
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

  return parseDetections(
    output_values, output_dims_, info, frame.cols, frame.rows,
    score_threshold, confidence_threshold, nms_threshold, yolo_variant);
}

void TensorRtYoloEngine::checkCuda(cudaError_t status, const char * operation)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
  }
}

void TensorRtYoloEngine::discoverTensors()
{
  for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
    const char * name = engine_->getIOTensorName(i);
    if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT && input_name_.empty()) {
      input_name_ = name;
    } else if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT && output_name_.empty()) {
      output_name_ = name;
    }
  }

  if (input_name_.empty() || output_name_.empty()) {
    throw std::runtime_error("TensorRT engine must expose one input and one output tensor.");
  }
}

void TensorRtYoloEngine::configureInputShape(int input_width, int input_height)
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

void TensorRtYoloEngine::allocateBuffers()
{
  input_dtype_ = engine_->getTensorDataType(input_name_.c_str());
  output_dtype_ = engine_->getTensorDataType(output_name_.c_str());
  if (input_dtype_ != nvinfer1::DataType::kFLOAT) {
    throw std::runtime_error("This app currently expects a float TensorRT input tensor.");
  }

  input_elements_ = getVolume(input_dims_);
  output_elements_ = getVolume(output_dims_);
  input_bytes_ = input_elements_ * elementSize(input_dtype_);
  output_bytes_ = output_elements_ * elementSize(output_dtype_);

  input_host_.assign(input_elements_, 0.0F);
  output_host_float_.assign(output_elements_, 0.0F);
  output_host_half_.assign(output_elements_, __float2half(0.0F));

  checkCuda(cudaMalloc(&device_input_, input_bytes_), "cudaMalloc input");
  checkCuda(cudaMalloc(&device_output_, output_bytes_), "cudaMalloc output");
  checkCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");

  if (!context_->setTensorAddress(input_name_.c_str(), device_input_)) {
    throw std::runtime_error("Failed to bind TensorRT input tensor memory.");
  }
  if (!context_->setTensorAddress(output_name_.c_str(), device_output_)) {
    throw std::runtime_error("Failed to bind TensorRT output tensor memory.");
  }
}

void TensorRtYoloEngine::fillInputBuffer(const cv::Mat & bgr_image)
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

std::vector<Detection> TensorRtYoloEngine::parseDetections(
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

    const cv::Rect scaled = scaleBoxToImage(
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

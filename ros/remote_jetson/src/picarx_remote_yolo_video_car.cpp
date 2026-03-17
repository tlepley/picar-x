#include "detection_pipeline.hpp"
#include "vehicle_controller.hpp"
#include "viewer_ui.hpp"

class RemoteJetsonVideoCarNode : public rclcpp::Node
{
public:
  RemoteJetsonVideoCarNode()
  : Node("picarx_remote_yolo_video_car")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/picarx/camera/image_raw");
    detection_pipeline_.configure(*this);
    vehicle_controller_.configure(*this);
    viewer_ui_.configure(*this);

    vehicle_controller_.initialize(*this);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RemoteJetsonVideoCarNode::onImage, this, std::placeholders::_1));

    viewer_ui_.initialize(*this);
    detection_pipeline_.initialize(*this);
    vehicle_controller_.centerCamera();
  }

  ~RemoteJetsonVideoCarNode() override
  {
    shutdown();
  }

  bool running() const
  {
    return running_;
  }

  void printHelp()
  {
    viewer_ui_.printHelp(*this);
  }

  void maybeRequestStreamOnStart()
  {
    vehicle_controller_.maybeRequestStreamOnStart(*this);
  }

  void shutdown()
  {
    if (!running_) {
      return;
    }
    running_ = false;
    vehicle_controller_.shutdown(*this);
    viewer_ui_.shutdown();
  }

  void uiTick()
  {
    cv::Mat frame;
    {
      std::scoped_lock<std::mutex> lock(frame_mutex_);
      if (!latest_frame_.empty()) {
        frame = latest_frame_.clone();
      }
    }

    if (frame.empty()) {
      viewer_ui_.renderPlaceholder(vehicle_controller_, detection_pipeline_);
      processViewerActions();
      return;
    }

    detection_pipeline_.processFrame(*this, frame);
    viewer_ui_.renderFrame(frame, vehicle_controller_, detection_pipeline_);
    processViewerActions();
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const auto decoded = decodeImage(*msg);
    if (!decoded.has_value()) {
      return;
    }
    std::scoped_lock<std::mutex> lock(frame_mutex_);
    latest_frame_ = decoded.value();
  }

  void processViewerActions()
  {
    for (const KeyAction action : viewer_ui_.pollActions(*this)) {
      switch (action) {
        case KeyAction::ToggleDetector:
          RCLCPP_INFO(
            get_logger(), "Detector %s",
            detection_pipeline_.toggleDetectorEnabled() ? "enabled" : "disabled");
          break;
        case KeyAction::Help:
          printHelp();
          break;
        case KeyAction::Quit:
          shutdown();
          break;
        case KeyAction::None:
          break;
        default:
          vehicle_controller_.applyAction(action);
          break;
      }
    }
  }

  std::string image_topic_;
  bool running_{true};
  DetectionPipeline detection_pipeline_;
  VehicleController vehicle_controller_;
  ViewerUi viewer_ui_;
  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RemoteJetsonVideoCarNode>();
  node->printHelp();
  node->maybeRequestStreamOnStart();

  while (rclcpp::ok() && node->running()) {
    rclcpp::spin_some(node);
    node->uiTick();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  node->shutdown();
  rclcpp::shutdown();
  return 0;
}

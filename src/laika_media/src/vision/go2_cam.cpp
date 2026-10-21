#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <thread>
#include <atomic>

class Go2CameraPublisher : public rclcpp::Node
{
public:
  Go2CameraPublisher()
  : Node("go2_camera_publisher"), running_(false)
  {
    // Parámetros
    this->declare_parameter("udp_address",    std::string("230.1.1.1"));
    this->declare_parameter("udp_port",       1720);
    this->declare_parameter("multicast_iface",std::string("eth0"));
    this->declare_parameter("topic",          std::string("go2/image_raw"));
    this->declare_parameter("frame_id",       std::string("camera"));

    const auto addr  = this->get_parameter("udp_address").as_string();
    const auto port  = this->get_parameter("udp_port").as_int();
    const auto iface = this->get_parameter("multicast_iface").as_string();
    topic_    = this->get_parameter("topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // Pipeline GStreamer para OpenCV
    /*
    pipeline_ =
      "udpsrc address=" + addr +
      " port=" + std::to_string(port) +
      " multicast-iface=" + iface +
      " auto-multicast=true"
      " ! queue"
      " ! application/x-rtp,media=video,encoding-name=H264"
      " ! rtph264depay ! h264parse ! avdec_h264"
      " ! videoconvert ! video/x-raw,format=BGR"
      " ! appsink drop=true max-buffers=1 sync=false" ;
*/

    pipeline_ =
	"udpsrc address=" + addr +
	" port=" + std::to_string(port) +
	" multicast-iface=" + iface +
	" auto-multicast=true"
	" caps=\"application/x-rtp, media=video, encoding-name=H264, clock-rate=90000\" "
	" ! rtpjitterbuffer latency=0 drop-on-latency=true"
	" ! rtph264depay ! h264parse ! avdec_h264"
	" ! videoconvert ! video/x-raw,format=BGR"
	" ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream"
	" ! appsink sync=false drop=true max-buffers=1";

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, 10);

    RCLCPP_INFO(get_logger(), "Topic   : %s", topic_.c_str());
    RCLCPP_INFO(get_logger(), "Pipeline: %s", pipeline_.c_str());

    running_ = true;
    capture_thread_ = std::thread(&Go2CameraPublisher::captureLoop, this);
  }

  ~Go2CameraPublisher()
  {
    running_ = false;
    if (capture_thread_.joinable())
      capture_thread_.join();
  }

private:
  void captureLoop()
  {
    cv::VideoCapture cap(pipeline_, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
      RCLCPP_ERROR(get_logger(), "No se pudo abrir el pipeline GStreamer.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Stream abierto. Publicando en %s ...", topic_.c_str());

    cv::Mat frame;
    while (running_ && rclcpp::ok()) {
      if (!cap.read(frame) || frame.empty()) {
        RCLCPP_WARN(get_logger(), "Frame vacío, reintentando...");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      auto msg = cv_bridge::CvImage(
        std_msgs::msg::Header{},
        "bgr8",
        frame
      ).toImageMsg();

      msg->header.stamp    = this->get_clock()->now();
      msg->header.frame_id = frame_id_;
      pub_->publish(*msg);
    }

    cap.release();
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::string pipeline_, topic_, frame_id_;
  std::thread capture_thread_;
  std::atomic<bool> running_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}

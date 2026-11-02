#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridg/cv_bridg.h>

// 🚧 [CLASS] 

class Go2Camera : public rclcpp::Node
{
	public:
		Go2Camera()
		: Node("go2_camera_publisher"), running_(false)
		{
			initP0();


		}

		~Go2Camera()
		{
			running_.store{true};

			// ❓[IF] THREAD JOINABLE
			if(capture_thread_.joinable()) 
				// ✅ [IF] TRUE
				capture_thread_.join();
		}

	private:

	// 🔑 [PROCESS 0] OPEN VIDEO PIPELINE STREAM
	
		// ⛛ [P0 VARS]
		std::string pipeline_;
		std::thread capture_thread_;
		std::atomic<bool> running_;


		// ▶ [INIT P0]
		void initP0()
		{
			this->declare_parameter<std::string>("udp_add", "230.1.1.1");
			this->declare_parameter("udp_port", 1720);
			this->declare_parameter<std::string>("multicast_iface", "eth0"); 

			pipeline_=
				" udpsrc address=" + addr +
				" port=" + std::to_string(port) +
				" multicast-iface=" + iface +
				" auto-multicast=true"
				" caps=\"aplication/x-rtp, media=video, encoding-name=H264, clock-rate=90000\" "
				" ! rtpjitterbuffer latency=0 drop-on-latency=true"
				" ! rtph264depay ! h264parse ! avdec_h264"
				" ! videoconvert ! video/x-raw,format=BGR"
				" ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream"
				" ! appsink sync=false drop=true max-buffers=1";

			capture_thread_ = std::thread(&Go2Camera::captureLoop, this);
		}

		// ☎ [CALLBACK P0]
		void captureLoop()
		{
			cv::VideoCapture cap(pipeline_, cv::CAP_GSTREAMER);

			// ❓[IF] CAP OPENED
			if (!cap.isOpened())
			{
				// ❌ [IF] CAP FAILED
				RCLCPP_ERROR(
					this->get_logger(),
					"❌ Failed opening Gstreamer pipeline."
					);
				return;
			}
			// ✅ [IF] CAP SUCCESS
			RCLCPP_INFO(
				this->get_logger(),
				" ✅ Success opening the streaming: \ "
				"	▶ Publishing at: %s",
				topic_.c_str()
				);










// ⚙ [LIB] C++ TOOLS
# include <atomic>
# include <iostream>
# include <chrono>
# include <functional>
# include <memory>

// ⚙ [LIB] ROS2
# include <rclcpp/rclcpp.hpp>
# include <sensor_msgs/msg/imu.hpp>
# include <geometry_msgs/msg/transform_stamped.hpp>

// ⚙ [LIB] GEOMETRY
# include <Eigen/Dense>
# include <sophus/so3.hpp>
# include <tf2_ros/static_transform_broadcaster.h>

// ⛑ [AUX SHORTCUTS]
using RosImuT = sensor_msgs::msg::Imu;
using RosTransformT = geometry_msgs::msg::TransformStamped;
using BroadcasterT = tf2_ros::StaticTransformBroadcaster;

// 🚧 [CLASS] GRAVITY ALIGNMENT
class GravityAlignment : public rclcpp::Node
{
	public:
		GravityAlignment() : Node{"lie_gravity_alignment"}
		{


		}
	private:

	// ⛑ [AUX]
		std::chrono::milliseconds getDuration(
						std::chrono::steady_clock::time_point first_timer,
						std::chrono::steady_clock::time_point last_timer)
		{
			return std::chrono::duration_cast<std::chrono::milliseconds>(last_timer - first_timer);
		}
	// 💡[FUN] 
	
		Eigen::Matrix3d globalIdentity(
				Eigen::Vector3d a)
		{
			Eigen


		}

	// ⛛ [VARS] ROS
		rclcpp::Subscription<RosImuT>::SharedPtr sub_;
		
	// ⛛ [VARS] OTHER
		std::atomic<bool> first_stamp_{false};
		std::chrono::steady_clock::time_point stamp_;
		int target_time_;
	
	// ⛛ [VARS] GEOMETRY
		Eigen::Vector3d samples_acc_;
		int count_;
	
	// ⛛ [VARS] RAII
		std::shared_ptr<BroadcasterT> tf_broadcaster_;

	// ▶ [INIT]
	
	void init()
	{
		// QoS
		rclcpp::QoS qos_policy(10);
		qos_policy.reliability(rclcpp::ReliabilityPolicy::BestEffort);
		qos_policy.durability(rclcpp::DurabilityPolicy::KeepLast);

		// RCLCPP SUBSCRIBER

		this->declare_parameters<int>("target_time", 3000);
		target_time_ = this->get_parameter("target_time").as_int();

		RCLCPP_INFO(
			this->get_logger(),
			" ✅ Timer set: %d \n",
			target_time_);


		auto lambda = [this](const RosImutT::SharedPtr msg){this->callbackF(msg); };

		this->declare_parameter<std::string>("imu_topic", "/livox/imu");
		const char* topic = this->get_parameter("imu_topic").as_string();

		sub_ = this->create_subscription<RosImuT>(
				topic,
				qos_profile,
				lambda);
		
		// VARS INITIALIZATION
		samples_acc_{0.0, 0.0, 0.0};
		count_ = 0;

		// STATIC TRANSFORM BROADCASTER
		tf_broadcaster_ = std::make_shared<BroadcasterT>(this);
	}

	// ☎ [CALLBACK]
	void callbackF(const RosImuT::SharedPtr m)
	{
		// ❓[IF] FIRST LOOP
		if(!first_stamp_.load())
		{
			stamp_ = std::chrono::steady_clock::now();
			int stamp_int = static_cast<int>(stamp_.count());
			RCLCPP_INFO(
				this->get_logger(),
				" ✅ [STEP 1] Stamped first cycle at: %d \n",
				stamp_int);

			first_stamp_.store(true);
		}

		// ⚡[OP] CALLBACK OPERATION

		std::chrono::steady_clock::time_point loop_time = std::chrono::steady_clock::now();
		std::chrono::milliseconds duration_uncon = getDuration(stamp_, loop_time);
		int duration = static_cast<int>(duration_uncon.count());

		// ❓[IF] SAMPLING TIME READY
		if (duration < target_time_)
		{
			// ❌ [IF] 
			++count_;
			samples_acc_.x() += m.linear_acceleration.x;
			samples_acc_.y() += m.linear_acceleration.y;
			samples_acc_.z() += m.linear_acceleration.z;
		}
		// ✅ [IF]
		Eigen::Vector3d a;
		a.x() = sampes_acc_.x() / count_ ;
		a.y() = sampes_acc_.y() / count_ ;
		a.z() = sampes_acc_.z() / count_ ;



















		
	
	


};





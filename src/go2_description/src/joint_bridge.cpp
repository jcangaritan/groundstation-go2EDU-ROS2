#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/low_state.hpp>

// 🚧 NODE RCLCPP CLASS
class Bridge : public rclcpp::Node
{
	public:
		Bridge() : Node("go2_joint_bridge")
	{
		initP0();

	}
	private:

	// ⛛ [VARS P0] ROS2 VARIABLES
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
	rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_;
	sensor_msgs::msg::JointState msg_;

	// ▶ [INIT P0]
	void initP0()
	{
		rclcpp::QoS qos_profile(10);
		qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
		qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

		auto topic = this->declare_parameter<std::string>("lowstate_topic", "/lf/lowstate");
		auto lambda = [this](unitree_go::msg::LowState::ConstSharedPtr m){this->cbP0(*m);};
		sub_ = this->create_subscription<unitree_go::msg::LowState>(
				topic,
				qos_profile,
				lambda);

		std::string pub_topic = "joint_states";
		pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
				pub_topic,
				qos_profile);

		// 💡FRONT RIGHT LEG
		std::string m0 = "FR_hip_joint";
		std::string m1 = "FR_thigh_joint";
		std::string m2 = "FR_calf_joint";

		// 💡FRONT LEFT LEG
		std::string m3 = "FL_hip_joint";
		std::string m4 = "FL_thigh_joint";
		std::string m5 = "FL_calf_joint";

		// 💡REAR RIGHT LEG
		std::string m6 = "RR_hip_joint";
		std::string m7 = "RR_thigh_joint";
		std::string m8 = "RR_calf_joint";

		// 💡 REAR LEFT LEG
		std::string m9 = "RL_hip_joint";
		std::string m10 = "RL_thigh_joint"; 
		std::string m11 = "RL_calf_joint";

		msg_.name = {m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11};

		msg_.position.resize(12);
		msg_.velocity.resize(12);
		msg_.effort.resize(12);
	}

	// ☎ [CALLBACK P0]
	void cbP0(const unitree_go::msg::LowState& s)
	{
		msg_.header.stamp = now();
		for(size_t i=0; i < 12; ++i)
		{
			msg_.position[i] = s.motor_state[i].q;
			msg_.velocity[i] = s.motor_state[i].dq;
			msg_.effort[i] = s.motor_state[i].tau_est;
		}
		pub_->publish(msg_);
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Bridge>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

// cruise_teleop_node.cpp - CAPA ROS FINA: mando Go2 -> eventos A/B -> CruiseController -> /cmd_vel.
// Toda la lógica vive en include/*.hpp (probada sin ROS). Aquí solo hay cableado.
// Nodo de un solo hilo (executor por defecto): los callbacks nunca se solapan -> sin mutex.
#include <algorithm>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

#include "button_edge.hpp"
#include "cruise_controller.hpp"
#include "go2_rc.hpp"

using namespace std::chrono_literals;
using go2rc::Button;
using RosTwistT    = geometry_msgs::msg::Twist;
using UnitreeWireT = unitree_go::msg::WirelessController;

class CruiseTeleop : public rclcpp::Node
{
public:
	CruiseTeleop() : Node("cruise_teleop_node")
	{
		const auto wireless_topic = declare_parameter<std::string>("wireless_topic", "/wirelesscontroller");
		const auto cmd_topic      = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
		rate_hz_       = declare_parameter<double>("publish_rate_hz", 20.0);
		ignore_chords_ = declare_parameter<bool>("ignore_when_triggers", true);
		debug_         = declare_parameter<bool>("debug", true);

		tele::CruiseParams cp;
		cp.vel        = declare_parameter<double>("cruise_vel", 0.6);
		cp.ramp.accel = declare_parameter<double>("ramp_accel", 0.5);
		cp.ramp.decel = declare_parameter<double>("ramp_decel", 1.0);
		cp.zero_tail  = declare_parameter<int>("zero_tail_ticks", 5);
		ctrl_ = tele::CruiseController(cp);

		btn::EdgeParams ep;
		ep.release_ns = static_cast<int64_t>(declare_parameter<double>("release_ms", 40.0) * 1e6);
		ep.lockout_ns = static_cast<int64_t>(declare_parameter<double>("lockout_ms", 200.0) * 1e6);
		edge_a_ = btn::EdgeDetector(ep);
		edge_b_ = btn::EdgeDetector(ep);

		pub_ = create_publisher<RosTwistT>(cmd_topic, 10);
		sub_ = create_subscription<UnitreeWireT>(wireless_topic, rclcpp::SensorDataQoS(),
			[this](UnitreeWireT::SharedPtr m) { onWireless(m->keys); });
		feed_timer_ = create_wall_timer(10ms, [this]() { feed(last_keys_); });   // rearma si el topic es on-change
		ctrl_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this]() { control(); });
		last_ctrl_ns_ = nowNs();

		RCLCPP_INFO(get_logger(), "[P0] CRUISE ready: A=ON B=OFF | %.2f m/s | %s -> %s 💡 \n",
			cp.vel, wireless_topic.c_str(), cmd_topic.c_str());
	}

private:
	int64_t nowNs() const { return std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count(); }

	void onWireless(uint32_t keys)
	{
		const int64_t t = nowNs();
		if (got_msg_ && debug_) {
			const double gap_ms = (t - last_msg_ns_) * 1e-6;
			if (gap_ms > 100.0) RCLCPP_WARN(get_logger(), "[P4] %.0f ms gap in controller messages, a tap may be lost ⚠ \n", gap_ms);
		}
		last_keys_ = keys; last_msg_ns_ = t; got_msg_ = true;
		feed(keys);                                   // a la velocidad del mensaje: ningún tap se pierde
	}

	void feed(uint32_t keys)
	{
		if (!got_msg_) return;
		const bool chord = ignore_chords_ && (go2rc::pressed(keys, Button::L2) || go2rc::pressed(keys, Button::R2));
		const bool raw_a = go2rc::pressed(keys, Button::A), raw_b = go2rc::pressed(keys, Button::B);
		const int64_t t = nowNs();
		const bool a = edge_a_.update(!chord && raw_a, t);
		const bool b = edge_b_.update(!chord && raw_b, t);

		if (debug_) {
			if (raw_a && !prev_a_ && !a) RCLCPP_WARN(get_logger(), "[P4] A press IGNORED: %s ⚠ \n", chord ? "L2/R2 chord" : "edge detector locked");
			if (raw_b && !prev_b_ && !b) RCLCPP_WARN(get_logger(), "[P4] B press IGNORED: %s ⚠ \n", chord ? "L2/R2 chord" : "edge detector locked");
		}
		prev_a_ = raw_a; prev_b_ = raw_b;

		if (b)      { if (ctrl_.enabled())  RCLCPP_INFO(get_logger(), "[P0] Machine State -> STOP 🛑 \n"); ctrl_.handle(tele::CruiseEvent::Stop); }
		else if (a) { if (!ctrl_.enabled()) RCLCPP_INFO(get_logger(), "[P0] Machine State -> CRUISE ⚡ \n");  ctrl_.handle(tele::CruiseEvent::Start); }
	}

	void control()
	{
		const int64_t t = nowNs();
		const double dt = std::clamp((t - last_ctrl_ns_) * 1e-9, 0.0, 0.2);   // dt medido, acotado
		last_ctrl_ns_ = t;
		const auto out = ctrl_.update(dt);
		if (!out.publish) return;
		RosTwistT m; m.linear.x = out.vx;
		pub_->publish(m);
	}

	double rate_hz_;
	bool ignore_chords_, debug_, got_msg_ = false, prev_a_ = false, prev_b_ = false;
	uint32_t last_keys_ = 0;
	int64_t last_msg_ns_ = 0, last_ctrl_ns_ = 0;
	btn::EdgeDetector edge_a_, edge_b_;
	tele::CruiseController ctrl_;
	rclcpp::Publisher<RosTwistT>::SharedPtr pub_;
	rclcpp::Subscription<UnitreeWireT>::SharedPtr sub_;
	rclcpp::TimerBase::SharedPtr feed_timer_, ctrl_timer_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CruiseTeleop>());
	rclcpp::shutdown();
	return 0;
}


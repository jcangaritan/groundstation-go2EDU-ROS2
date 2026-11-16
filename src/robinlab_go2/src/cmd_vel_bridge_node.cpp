// cmd_vel_bridge_node.cpp - CAPA ROS FINA: /cmd_vel -> BridgeLogic -> SportClient (Unitree).
// La decisión (sanear, limitar, Move/Stop, watchdog de una sola vez) está en bridge_logic.hpp.
// Un solo hilo: el callback de cmd_vel y el timer del watchdog nunca se solapan -> sin mutex.
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "common/ros2_sport_client.h"
#include "bridge_logic.hpp"

using RosTwistT = geometry_msgs::msg::Twist;
using namespace std::chrono_literals;

class CmdVelBridge : public rclcpp::Node
{
public:
	CmdVelBridge() : Node("cmd_vel_bridge"), sport_(this)
	{
		tele::BridgeLimits l;
		l.max_vx = declare_parameter<double>("max_vx", 3.0);
		l.max_vy = declare_parameter<double>("max_vy", 1.0);
		l.max_wz = declare_parameter<double>("max_wz", 1.0);
		l.timeout_ns = static_cast<int64_t>(declare_parameter<double>("cmd_timeout_s", 0.5) * 1e9);
		const auto topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
		logic_ = tele::BridgeLogic(l);

		sub_ = create_subscription<RosTwistT>(topic, rclcpp::QoS(rclcpp::KeepLast(10)),
			[this](RosTwistT::SharedPtr m) {
				const auto a = logic_.onCmd(m->linear.x, m->linear.y, m->angular.z, nowNs());

				// ❓[P0][CMD_VEL CORRUPT (NaN / inf)]
				if(logic_.rejected() != rejected_seen_)
				{
					// ❌ [P0][CMD_VEL CORRUPT] -> STOP
					rejected_seen_ = logic_.rejected();
					RCLCPP_WARN(
						this->get_logger(),
						"[P0] cmd_vel with NaN/inf, forcing STOP ⚠ \n");
				}
				apply(a);
			});
		timer_ = create_wall_timer(100ms, [this]() {
			const auto a = logic_.onTick(nowNs());
			// ❓[P1][WATCHDOG TIMEOUT]
			if(a.kind == tele::Action::Stop)
			{
				// ✅ [P1][WATCHDOG TIMEOUT] -> StopMove (ONE SHOT)
				RCLCPP_WARN(
					this->get_logger(),
					"[P1] No cmd_vel within timeout, StopMove sent 🛑 \n");
			}
			apply(a);
		});
	}

private:
	// ⛑ [AUX] CLOCK: steady_clock (monotonic). NOT high_resolution_clock: in libstdc++ it is system_clock,
	// so an NTP/GNSS time jump could fire the watchdog by mistake or, worse, keep it from ever firing.
	int64_t nowNs() const { return std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count(); }

	void apply(const tele::Action& a)           // ÚNICO punto que toca el SDK de Unitree
	{
		switch (a.kind) {
			case tele::Action::Move: sport_.Move(req_, a.vx, a.vy, a.wz); break;
			case tele::Action::Stop: sport_.StopMove(req_); break;
			case tele::Action::None: break;
		}
	}

	SportClient sport_;
	unitree_api::msg::Request req_;
	tele::BridgeLogic logic_;
	unsigned rejected_seen_ = 0;
	rclcpp::Subscription<RosTwistT>::SharedPtr sub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CmdVelBridge>());
	rclcpp::shutdown();
	return 0;
}


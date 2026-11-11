#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>

#include "gravity_aligner.hpp"

using RosImuT = sensor_msgs::msg::Imu;
using BroadcasterT = tf2_ros::StaticTransformBroadcaster;

class GravityAlignment : public rclcpp::Node {
 public:
  GravityAlignment() : Node("lie_gravity_alignment") {
    const std::string topic = declare_parameter<std::string>("imu_topic", "/livox/imu");
    parent_ = declare_parameter<std::string>("parent_frame", "gravity_world");
    child_  = declare_parameter<std::string>("child_frame", "camera_init");

    gravity::GravityAligner::Params p;
    p.expected_norm = declare_parameter<double>("expected_norm", 1.0);   // 1.0 = g, 9.81 = m/s^2
    p.norm_tol      = declare_parameter<double>("norm_tol", 0.05);
    p.max_gyro      = declare_parameter<double>("max_gyro", 0.05);       // rad/s
    p.max_std       = declare_parameter<double>("max_std", 0.01);
    p.n_samples     = static_cast<std::size_t>(declare_parameter<int>("n_samples", 400));
    aligner_ = gravity::GravityAligner(p);

    sub_ = create_subscription<RosImuT>(
        topic, rclcpp::SensorDataQoS(),
        [this](const RosImuT::SharedPtr m) { callback(m); });
    tf_broadcaster_ = std::make_shared<BroadcasterT>(this);

    RCLCPP_INFO(get_logger(), "Waiting for %zu still samples on %s", p.n_samples, topic.c_str());
  }

 private:
  void callback(const RosImuT::SharedPtr m) {
    if (aligner_.done()) return;

    const Eigen::Vector3d a(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
    const Eigen::Vector3d w(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
    if (!aligner_.addSample(a, w)) return;

    // R = ε ⊞ δθ = Exp(δθ)
    const Eigen::Quaterniond q = aligner_.R_wb().unit_quaternion();
    const double tilt_deg = aligner_.deltaTheta().norm() * 180.0 / M_PI;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = parent_;
    tf.child_frame_id = child_;
    tf.transform.translation.x = tf.transform.translation.y = tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);

    RCLCPP_INFO(get_logger(), "Aligned: tilt %.3f deg, |a| std %.2f mg. Published %s -> %s",
                tilt_deg, 1e3 * aligner_.stdAccel().maxCoeff(), parent_.c_str(), child_.c_str());
    sub_.reset();  // nothing more to do
  }

  gravity::GravityAligner aligner_;
  std::string parent_, child_;
  rclcpp::Subscription<RosImuT>::SharedPtr sub_;
  std::shared_ptr<BroadcasterT> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GravityAlignment>());
  rclcpp::shutdown();
  return 0;
}

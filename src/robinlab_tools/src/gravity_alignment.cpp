// gravity_alignment_node.cpp
//
// Wires the two components together and talks to ROS 2:
//   AccelerometerSampler (accelerometer_sampler.hpp): decides which IMU data is valid.
//   gravityIncrement + perturbIdentity (gravity_alignment.hpp): computes R_wb = eps (+) delta_theta.
// When the sampling window is valid it publishes ONE static TF  parent_frame -> child_frame
// (default: gravity_world -> camera_init) and stops listening.
//
// NOTE: not compiled in the sandbox where it was written (no ROS 2 there). The two headers it uses
// are covered by unit tests.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>

#include "accelerometer_sampler.hpp"
#include "gravity_alignment.hpp"

using RosImuT = sensor_msgs::msg::Imu;
using BroadcasterT = tf2_ros::StaticTransformBroadcaster;

class GravityAlignment : public rclcpp::Node {
 public:
  GravityAlignment() : Node("lie_gravity_alignment") {
    const std::string topic = declare_parameter<std::string>("imu_topic", "/livox/imu");
    parent_ = declare_parameter<std::string>("parent_frame", "gravity_world");
    child_ = declare_parameter<std::string>("child_frame", "camera_init");

    gravity::AccelerometerSampler::Params p;
    p.expected_norm = declare_parameter<double>("expected_norm", 1.0);  // 1.0 = g, 9.81 = m/s^2
    p.norm_tol = declare_parameter<double>("norm_tol", 0.05);
    p.max_gyro = declare_parameter<double>("max_gyro", 0.0);            // <= 0: accelerometer only
    p.max_std = declare_parameter<double>("max_std", 0.05);             // relative to expected_norm
    p.n_samples = static_cast<std::size_t>(declare_parameter<int>("n_samples", 400));
    n_target_ = p.n_samples;

    // The sampler stays ROS-free; its optional log callback is routed to RCLCPP_INFO here.
    sampler_ = gravity::AccelerometerSampler(
        p, [this](const std::string& msg) { RCLCPP_INFO(get_logger(), "%s", msg.c_str()); });

    sub_ = create_subscription<RosImuT>(
        topic, rclcpp::SensorDataQoS(), [this](const RosImuT::SharedPtr m) { callback(m); });
    tf_broadcaster_ = std::make_shared<BroadcasterT>(this);

    RCLCPP_INFO(get_logger(), "[SAMPLING IMU] waiting for %zu valid samples on %s", n_target_,
                topic.c_str());
  }

 private:
  void callback(const RosImuT::SharedPtr m) {
    if (!topic_detected_) {
      topic_detected_ = true;
      RCLCPP_INFO(get_logger(), "[SAMPLING IMU] : topic detected (frame_id: %s)",
                  m->header.frame_id.c_str());
    }
    if (sampler_.ready()) return;

    const Eigen::Vector3d a(m->linear_acceleration.x, m->linear_acceleration.y,
                            m->linear_acceleration.z);
    const Eigen::Vector3d w(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);

    // Component 1: sampling / rejection.
    const gravity::SampleStatus status = sampler_.addSample(a, w);

    const auto& s = sampler_.stats();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "[SAMPLING IMU] n=%zu/%zu | seen=%zu rej(gyro=%zu norm=%zu std=%zu) | "
                         "|a|=%.3f |w|=%.4f last_std=%.4f",
                         sampler_.count(), n_target_, s.samples, s.rej_gyro, s.rej_norm, s.rej_std,
                         s.last_norm, s.last_gyro, s.last_std);
    if (status != gravity::SampleStatus::Ready) return;

    // Component 2: alignment.
    //   step 1: delta_theta in the tangent space (direction, axis, angle, composition)
    //   step 2: perturb the identity: R = eps (+) delta_theta
    const Eigen::Vector3d delta_theta = gravity::gravityIncrement(sampler_.mean());
    const Sophus::SO3d R_wb = gravity::perturbIdentity(delta_theta);
    const Eigen::Quaterniond q = R_wb.unit_quaternion();

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

    RCLCPP_INFO(get_logger(), "[ALIGNMENT] tilt %.3f deg, std %.2f mg. Published %s -> %s",
                delta_theta.norm() * 180.0 / M_PI, 1e3 * sampler_.stdDev().maxCoeff(),
                parent_.c_str(), child_.c_str());
    sub_.reset();  // done; the static TF stays latched
  }

  gravity::AccelerometerSampler sampler_;
  std::size_t n_target_ = 0;
  bool topic_detected_ = false;
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

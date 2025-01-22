#include "vslam_px4_bridge/frame_transforms.hpp"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace frame_transforms;

class VslamPX4Bridge : public rclcpp::Node {
public:
  VslamPX4Bridge() : Node("vslam_px4_bridge") {

    std::string px4_pub_name = "/px4_7/fmu/in/vehicle_visual_odometry";
    std::string vslam_sub_name = "/visual_slam/tracking/odometry";

    RCLCPP_INFO(this->get_logger(), "Using Vslam -> Visual Odometry callback!");

    position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        vslam_sub_name, 1, std::bind(&VslamPX4Bridge::callback, this, _1));

    odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        px4_pub_name, 10);

    RCLCPP_INFO(get_logger(), "Subscribed to %s",
                position_sub_->get_topic_name());
    RCLCPP_INFO(get_logger(), "Publishing to %s",
                odometry_pub_->get_topic_name());
  }

private:
  // publishers
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_pub_;

  // subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;

  bool got_first_message = false;
  uint8_t counter_ = 0;

  // last message
  Eigen::Vector3d last_pos;

  void callback(const nav_msgs::msg::Odometry &vslam_msg) {

    constexpr int downsample = 2;

    // check if we need to skip the message
    counter_++;
    if (!(counter_ % downsample == 0)) {
      return;
    }

    // construct the odom message
    auto px4_msg = px4_msgs::msg::VehicleOdometry();

    // get the timestamp
    uint64_t sample_microseconds =
        vslam_msg.header.stamp.sec * 1e6 + vslam_msg.header.stamp.nanosec * 1e3;

    px4_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    px4_msg.timestamp_sample = sample_microseconds;

    // pose frame
    px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    // get the eigen version of the odometry message
    Eigen::Vector3d p_enu(vslam_msg.pose.pose.position.x,
                          vslam_msg.pose.pose.position.y,
                          vslam_msg.pose.pose.position.z);

    Eigen::Quaterniond q_enu(
        vslam_msg.pose.pose.orientation.w, vslam_msg.pose.pose.orientation.x,
        vslam_msg.pose.pose.orientation.y, vslam_msg.pose.pose.orientation.z);

    if (!got_first_message) {
      last_pos = p_enu;
      got_first_message = true;
      return;
    }

    // check if it is safe to forward the message
    if ((p_enu - last_pos).norm() > 0.5) {
      // MOVED TOO MUCH!
      // EXIT
      RCLCPP_ERROR(get_logger(), "VSLAM JUMPED by %f m. Exiting...",
                   (p_enu - last_pos).norm());
      rclcpp::shutdown();
    }

    // get the NED versions
    Eigen::Vector3d p_ned(p_enu[1], p_enu[0], -p_enu[2]);

    // extract the yaw
    double yaw = utils::quaternion::quaternion_get_yaw(q_enu);

    // convert to quaternion
    double yaw_ned = -yaw + M_PI / 2.0;
    Eigen::Quaterniond q_ned(std::cos(yaw_ned / 2.0), 0.0, 0.0,
                             std::sin(yaw_ned / 2.0));

    // // transform to NED
    // Eigen::Vector3d p_ned = transform_static_frame(p_enu,
    // StaticTF::ENU_TO_NED); Eigen::Quaterniond q_ned =
    // transform_orientation(q_enu, StaticTF::ENU_TO_NED);

    // RCLCPP_INFO(get_logger(), "ENU POS: %f, %f, %f", p_enu[0], p_enu[1],
    // p_enu[2]); RCLCPP_INFO(get_logger(), "NED POS: %f, %f, %f", p_ned[0],
    // p_ned[1], p_ned[2]);

    // set px4 message
    for (uint8_t i = 0; i < 3; ++i) {
      px4_msg.position[i] = p_ned[i];
    }
    px4_msg.q[0] = q_ned.w();
    px4_msg.q[1] = q_ned.x();
    px4_msg.q[2] = q_ned.y();
    px4_msg.q[3] = q_ned.z();

    // TODO(dev): also pass in the velocity?

    // velocity frame
    px4_msg.velocity_frame =
        px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;

    for (uint8_t i = 0; i < 3; i++) {
      px4_msg.velocity[i] = NAN;
      px4_msg.angular_velocity[i] = NAN;
    }

    // position covariance
    for (size_t i = 0; i < 3; i++) {
      px4_msg.position_variance[i] = 0.01;
      px4_msg.orientation_variance[i] = 0.0523; // approx 3 degrees
      px4_msg.velocity_variance[i] = NAN;
    }

    px4_msg.reset_counter = 0;

    odometry_pub_->publish(px4_msg);

    // set the first message flag
    last_pos = p_enu;
  }
};

int main(int argc, char *argv[]) {
  std::cout << "Starting vslam px4 bridge node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VslamPX4Bridge>());

  rclcpp::shutdown();
  return 0;
}

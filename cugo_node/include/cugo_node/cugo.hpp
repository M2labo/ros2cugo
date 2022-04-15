#ifndef CUGO_NODE__TURTLEBOT3_HPP_
#define CUGO_NODE__TURTLEBOT3_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <JetsonGPIO.h>

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>


namespace m2labo
{
namespace cugo
{
class Cugo : public rclcpp::Node
{
public:

  explicit Cugo(GPIO::PWM & pwm_right,GPIO::PWM & pwm_left);
  virtual ~Cugo() {}

private:

  void run(GPIO::PWM & pwm_right,GPIO::PWM & pwm_left);

  void publish_timer(const std::chrono::milliseconds timeout,GPIO::PWM & pwm_right,GPIO::PWM & pwm_left);
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  bool calculate_odometry_PWM(const rclcpp::Duration & duration);
  void publish(const rclcpp::Time & now);
  
  float v;
  float w;
  


  std::unique_ptr<nav_msgs::msg::Odometry> odom_;

  rclcpp::Node::SharedPtr node_handle_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Time now;
  rclcpp::Time last_time;

  std::array<double, 3> robot_pose_;
  std::array<double, 3> robot_vel_;

};
}  // namespace cugo
}  // namespace m2labo
#endif  // CUGO_NODE__TURTLEBOT3_HPP_

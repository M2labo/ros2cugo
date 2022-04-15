#include "cugo_node/cugo.hpp"

#include <memory>
#include <string>
#include <thread>

using m2labo::cugo::Cugo;
using namespace std::chrono_literals;
using namespace GPIO;
using std::this_thread::sleep_for;

auto val_r = 7.5;
auto val_l = 7.5;

Cugo::Cugo(GPIO::PWM & pwm_right,GPIO::PWM & pwm_left)
: Node("cugo_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init Cugo Node Main");
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  run(pwm_right,pwm_left);

}


void Cugo::run(GPIO::PWM & pwm_right,GPIO::PWM & pwm_left)
{
  RCLCPP_INFO(this->get_logger(), "Run!");
  v=0.0;
  w=0.0;

  publish_timer(std::chrono::milliseconds(50),pwm_right,pwm_left);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    qos,
    std::bind(&Cugo::cmd_callback, this, std::placeholders::_1));
}


void Cugo::publish_timer(const std::chrono::milliseconds timeout,GPIO::PWM & pwm_right,GPIO::PWM & pwm_left)
{
  publish_timer_ = this->create_wall_timer(
    timeout,
    [this,&pwm_right,&pwm_left]() -> void
    {
      rclcpp::Clock ros_clock(RCL_ROS_TIME);
      now = ros_clock.now();
      rclcpp::Duration duration(rclcpp::Duration::from_nanoseconds(
          now.nanoseconds() - last_time.nanoseconds()));
      
      calculate_odometry_PWM(duration);
      pwm_right.ChangeDutyCycle(val_r);
      pwm_left.ChangeDutyCycle(val_l);
      publish(now);
      last_time=now;
    }
  );
}

void Cugo::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "cmd_vel %f",msg->linear.x);
  v=msg->linear.x;
  w=msg->angular.z;
}


void Cugo::publish(const rclcpp::Time & now)
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id = "base_link";
  odom_msg->header.stamp = now;

  odom_msg->pose.pose.position.x = robot_pose_[0];
  odom_msg->pose.pose.position.y = robot_pose_[1];
  odom_msg->pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, robot_pose_[2]);

  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

  odom_msg->twist.twist.linear.x = robot_vel_[0];
  odom_msg->twist.twist.angular.z = robot_vel_[2];

  geometry_msgs::msg::TransformStamped odom_tf;

  odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
  odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.header.stamp = now;

  odom_pub_->publish(std::move(odom_msg));

  tf_broadcaster_->sendTransform(odom_tf);
}



bool Cugo::calculate_odometry_PWM(const rclcpp::Duration & duration)
{

  double delta_s = 0.0;
  double delta_theta = 0.0;

  double step_time = duration.seconds();

  // compute odometric instantaneouse velocity
  delta_s = v* step_time;
  delta_theta =w* step_time;
  
  if (step_time == 0.0) {
    return false;
  }


  // compute odometric pose
  robot_pose_[0] += delta_s * cos(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[1] += delta_s * sin(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[2] += delta_theta;

  robot_vel_[0] = v;
  robot_vel_[1] = 0.0;
  robot_vel_[2] = w;

  float wheel_r=(2.0*v-0.33*w)/2.0;
  float wheel_l= 2.0*v -  wheel_r;
  if (wheel_r >0.0){
    val_r = 8.0+(wheel_r/3.0)*2.5;
  }else if(wheel_r <0.0){
    val_r = 7.0+(wheel_r/3.0)*2.5;
  }else{
    val_r = 7.5;
  }
  if (wheel_l >0.0){
    val_l = 8.0+(wheel_l/3.0)*2.5;
  }else if(wheel_l <0.0){
    val_l = 7.0+(wheel_l/3.0)*2.5;
  }else{
    val_l = 7.5;
  }
  
  RCLCPP_INFO(this->get_logger(), "x : %f, y : %f", val_r, val_l);
  return true;
}
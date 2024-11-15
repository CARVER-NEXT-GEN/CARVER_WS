// src/odom_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "steering_controllers_library/steering_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "amt212ev_interfaces/msg/amt_read.hpp"

class OdomNode : public rclcpp::Node
{
public:
  OdomNode()
  : Node("odom_node"),
    odometry_(/* rolling_window_size */ 10)
  {
    // Declare and get parameters
    this->declare_parameter<double>("wheel_separation", 0.5);
    this->declare_parameter<double>("wheel_radius", 0.1);
    this->declare_parameter<double>("steering_radius", 0.1);
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("base_frame_id", "base_link");

    this->get_parameter("wheel_separation", wheel_separation_);
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("steering_radius", steering_radius_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("base_frame_id", base_frame_id_);

    // Configure odometry
    odometry_.set_wheel_params(wheel_separation_, wheel_radius_, steering_radius_);
    odometry_.set_odometry_type(/* odometry_type */ 0); // Adjust as needed

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&OdomNode::cmd_vel_callback, this, std::placeholders::_1));
    amt_read_sub_ = this->create_subscription<amt212ev_interfaces::msg::AmtRead>(
      "amt_publisher", 10, std::bind(&OdomNode::amt_callback, this, std::placeholders::_1)
    );

    // Timer for periodic updates
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&OdomNode::update_odometry, this));

    last_time_ = this->now();
  }

private:
  void amt_callback(const amt212ev_interfaces::msg::AmtRead::SharedPtr msg)
  {
    last_angular_velocity_ = msg->radps;
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Store the latest command velocities
    last_linear_velocity_ = msg->linear.x;
    // last_angular_velocity_ = msg->angular.z;
  }

  void update_odometry()
  {
    // Calculate time elapsed
    rclcpp::Time current_time = this->now();
    rclcpp::Duration period = current_time - last_time_;
    last_time_ = current_time;

    // Update odometry
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());

    // Create odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    // Set position
    odom_msg.pose.pose.position.x = odometry_.get_x();
    odom_msg.pose.pose.position.y = odometry_.get_y();
    odom_msg.pose.pose.position.z = 0.0;

    // Set orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, odometry_.get_heading());
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    // Set velocities
    odom_msg.twist.twist.linear.x = odometry_.get_linear();
    odom_msg.twist.twist.angular.z = odometry_.get_angular();

    // Publish odometry message
    odom_pub_->publish(odom_msg);
  }

  // Node variables
  steering_odometry::SteeringOdometry odometry_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<amt212ev_interfaces::msg::AmtRead>::SharedPtr amt_read_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double wheel_separation_;
  double wheel_radius_;
  double steering_radius_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  // Last velocities
  double last_linear_velocity_ = 0.0;
  double last_angular_velocity_ = 0.0;

  rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

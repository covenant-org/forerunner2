#include "controller.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/create_client.hpp>

namespace SimplePlanner {
Controller::Controller(std::string px4_namespace)
    : rclcpp::Node("SimplePlannerController") {
  this->declare_parameter("goal_tolerance", 0.5);
  this->declare_parameter("max_distance", 20.0);
  this->declare_parameter("enforce_max_distance", true);
  this->declare_parameter("min_height", 1.5);
  this->declare_parameter("reached_subscription_topic", "/reached_position");
  this->declare_parameter("smooth_weight_data", 0.5);
  this->declare_parameter("smooth_weight_smooth", 0.1);
  this->declare_parameter("smooth_tolerance", 1.0);
  this->declare_parameter("step_size", 1.0);
  this->declare_parameter("autorequest", true);
  this->declare_parameter("yaw_tolerance", 0.3);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
  this->path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", qos,
      std::bind(&Controller::path_cb, this, std::placeholders::_1));
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos_px4 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                             qos_profile);
  this->client_ptr_ =
      rclcpp_action::create_client<ReplanAction>(this, "replan");
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }
  this->goal_options.goal_response_callback = std::bind(
      &Controller::goal_response_callback, this, std::placeholders::_1);
  this->goal_options.feedback_callback =
      std::bind(&Controller::feedback_callback, this, std::placeholders::_1,
                std::placeholders::_2);
  this->goal_options.result_callback =
      std::bind(&Controller::result_callback, this, std::placeholders::_1);

  this->local_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/vehicle_pose", qos_px4,
          std::bind(&Controller::local_pose_cb, this, std::placeholders::_1));
  this->reached_position_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          this->get_parameter("reached_subscription_topic").as_string(), 10);

  this->offboard_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
          px4_namespace + "/in/offboard_control_mode", 10);
  this->trajectory_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
          px4_namespace + "/in/trajectory_setpoint", 10);
  this->trajectory_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
  this->vehicle_command_pub_ =
      this->create_publisher<px4_msgs::msg::VehicleCommand>(
          px4_namespace + "/in/vehicle_command", 10);
  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  this->smoothed_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/smoothed_path", 10);

  this->home_pos_sub_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "/home_position", 10,
          [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            this->home_position(0) = msg->point.x;
            this->home_position(1) = msg->point.y;
            this->home_position(2) = msg->point.z;
          });

  this->index = 0;
  this->sent_point = 0;
  this->recived_path = false;
  this->recorded_initial_position = false;
  this->vehicle_initial_position.x = 0.0;
  this->vehicle_initial_position.y = 0.0;
  this->vehicle_initial_position.z = 0.0;
  this->timer_ =
      this->create_wall_timer(100ms, std::bind(&Controller::control, this));
  this->debauncer = 0;
  RCLCPP_INFO(this->get_logger(), "Init controller node");
}

void Controller::local_pose_cb(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  this->current_position = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  if (!this->waiting_reponse && this->recived_path &&
      this->index < this->path.poses.size()) {
    Eigen::Vector3d current_position(msg->pose.position.x, msg->pose.position.y,
                                     msg->pose.position.z);
    Eigen::Quaterniond current_orientation(
        msg->pose.orientation.w, msg->pose.orientation.x,
        msg->pose.orientation.y, msg->pose.orientation.z);
    double dist = std::sqrt((current_position - this->temp_goal).squaredNorm());
    double tolerance = this->get_parameter("goal_tolerance").as_double();
    RCLCPP_DEBUG(this->get_logger(), "Distance to goal: %f, tolerance: %f",
                 dist, tolerance);
    double angular_dist =
        current_orientation.angularDistance(this->temp_orientation);
    RCLCPP_DEBUG(this->get_logger(), "Orientation to goal: %f", angular_dist);
    bool autorequest = this->get_parameter("autorequest").as_bool();
    double yaw_tolerance = this->get_parameter("yaw_tolerance").as_double();
    if (autorequest && dist < tolerance && angular_dist < yaw_tolerance) {
      //    this->reached_position_pub_->publish(*msg);
      ReplanAction::Goal replan_msg;
      replan_msg.pose = *msg;
      replan_msg.current_path_index = this->index;
      this->waiting_reponse = true;
      this->client_ptr_->async_send_goal(replan_msg, this->goal_options);
    }
  }

  if (this->recorded_initial_position) {
    return;
  }
  this->vehicle_initial_position.x = msg->pose.position.x;
  this->vehicle_initial_position.y = msg->pose.position.y;
  this->vehicle_initial_position.z = msg->pose.position.z;
  this->recorded_initial_position = true;
  RCLCPP_INFO(this->get_logger(), "Recorded initial position");
}

void Controller::smooth_path() {
  const double weight_data =
      this->get_parameter("smooth_weight_data").as_double();
  const double weight_smooth =
      this->get_parameter("smooth_weight_smooth").as_double();
  const double tolerance = this->get_parameter("smooth_tolerance").as_double();

  nav_msgs::msg::Path smoothed_path(this->path);
  double change = tolerance;
  RCLCPP_DEBUG(this->get_logger(), "Smoothing path");
  while (change >= tolerance) {
    change = 0.0;
    for (size_t i = 1; i < smoothed_path.poses.size() - 1; i++) {
      Eigen::Vector3d original_position(this->path.poses[i].pose.position.x,
                                        this->path.poses[i].pose.position.y,
                                        this->path.poses[i].pose.position.z);
      Eigen::Vector3d current_position(smoothed_path.poses[i].pose.position.x,
                                       smoothed_path.poses[i].pose.position.y,
                                       smoothed_path.poses[i].pose.position.z);
      Eigen::Vector3d next_position(smoothed_path.poses[i + 1].pose.position.x,
                                    smoothed_path.poses[i + 1].pose.position.y,
                                    smoothed_path.poses[i + 1].pose.position.z);
      Eigen::Vector3d prev_position(smoothed_path.poses[i - 1].pose.position.x,
                                    smoothed_path.poses[i - 1].pose.position.y,
                                    smoothed_path.poses[i - 1].pose.position.z);
      Eigen::Vector3d smoothed =
          original_position +
          weight_data * (original_position - current_position) +
          weight_smooth *
              (next_position + prev_position - 2.0 * current_position);
      change += Eigen::abs((original_position - smoothed).array()).sum();
      RCLCPP_DEBUG(this->get_logger(),
                   "Change: %f, original: %f %f %f smoothed: %f %f %f", change,
                   original_position(0), original_position(1),
                   original_position(2), smoothed(0), smoothed(1), smoothed(2));
      smoothed_path.poses[i].pose.position.x = smoothed(0);
      smoothed_path.poses[i].pose.position.y = smoothed(1);
      smoothed_path.poses[i].pose.position.z = smoothed(2);
    }
    RCLCPP_DEBUG(this->get_logger(), "Change: %f", change);
  }
  RCLCPP_DEBUG(this->get_logger(), "Path smoothed");
  this->smoothed_path_pub_->publish(smoothed_path);
  this->path = smoothed_path;
}

void Controller::path_cb(const nav_msgs::msg::Path::SharedPtr path) {
  this->waiting_reponse = false;
  if (path->poses.size() == 0) return;
  auto first_pose = path->poses[0];
  Eigen::Vector3d first_coord(first_pose.pose.position.x,
                              first_pose.pose.position.y,
                              first_pose.pose.position.z);
  this->path = nav_msgs::msg::Path(*path);
  this->trajectory_pub_->publish(this->path);
  // this->smooth_path();
  this->index = std::min(1, int(path->poses.size() - 1));

  auto pose = this->path.poses[this->index];
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform("map", pose.header.frame_id,
                                    tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                pose.header.frame_id.c_str(), "map", ex.what());
    return;
  }

  tf2::Quaternion tf_quat(t.transform.rotation.x, t.transform.rotation.y,
                          t.transform.rotation.z, t.transform.rotation.w);
  tf2::Vector3 tf_vec(t.transform.translation.x, t.transform.translation.y,
                      t.transform.translation.z);
  this->initial_transform = tf2::Transform(tf_quat, tf_vec);
  tf2::Vector3 start_point(first_coord(0), first_coord(1), first_coord(2));
  start_point = this->initial_transform * start_point;
  this->last_path_start_position =
      Eigen::Vector3d(start_point.x(), start_point.y(), start_point.z());
  this->recived_path = true;
}

void Controller::control() {
  if (!this->recived_path) {
    return;
  }

  if (sent_point == 10) {
    this->publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    this->arm();
  }

  this->publish_offboard_control_mode();

  if (sent_point < 10) {
    px4_msgs::msg::TrajectorySetpoint setpoint_msg{};
    setpoint_msg.position = {this->vehicle_initial_position.x,
                             -this->vehicle_initial_position.y,
                             -this->vehicle_initial_position.z};
    setpoint_msg.yaw = this->vehicle_initial_position.heading;
    setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->trajectory_setpoint_pub_->publish(setpoint_msg);
  } else {
    // TODO: Check if path is not empty and pose is valid
    auto pose = path.poses.at(this->index);
    this->publish_trajectory_setpoint(pose);
  }
  this->sent_point += 1;

  RCLCPP_DEBUG(this->get_logger(), "Publish point");
}

void Controller::publish_trajectory_setpoint(
    geometry_msgs::msg::PoseStamped &pose) {
  px4_msgs::msg::TrajectorySetpoint setpoint_msg{};

  tf2::Vector3 point(pose.pose.position.x, pose.pose.position.y,
                     pose.pose.position.z);

  tf2::Vector3 transformed = this->initial_transform * point;
  double min_height = this->get_parameter("min_height").as_double();
  if (transformed.z() < min_height) {
    transformed.setZ(min_height);
  }

  double step_size = this->get_parameter("step_size").as_double();
  Eigen::Vector3d ePoint(transformed.x(), transformed.y(), transformed.z());

  double distance_to_point = (ePoint - this->last_path_start_position).norm();
  RCLCPP_DEBUG(this->get_logger(), "Distance to point: %f", distance_to_point);
  if (distance_to_point > step_size) {
    RCLCPP_DEBUG(this->get_logger(), "Current point %f, %f, %f",
                 this->last_path_start_position(0),
                 this->last_path_start_position(1),
                 this->last_path_start_position(2));
    RCLCPP_DEBUG(this->get_logger(), "Original point %f, %f, %f", ePoint(0),
                 ePoint(1), ePoint(2));
    double percentaje = step_size / distance_to_point;
    transformed = tf2::Vector3(
        (this->last_path_start_position(0) +
         (ePoint(0) - this->last_path_start_position(0)) * percentaje),
        (this->last_path_start_position(1) +
         (ePoint(1) - this->last_path_start_position(1)) * percentaje),
        (this->last_path_start_position(2) +
         (ePoint(2) - this->last_path_start_position(2)) * percentaje));
    RCLCPP_DEBUG(this->get_logger(), "Modified %f, %f, %f", transformed.x(),
                 transformed.y(), transformed.z());
  }

  this->temp_goal =
      Eigen::Vector3d(transformed.x(), transformed.y(), transformed.z());
  bool enforce_max_distance =
      this->get_parameter("enforce_max_distance").as_bool();
  if (enforce_max_distance) {
    double max_distance = this->get_parameter("max_distance").as_double();
    ePoint = Eigen::Vector3d(transformed.x(), transformed.y(), transformed.z());
    double distance = (ePoint - this->home_position).norm();
    if (distance > max_distance) {
      RCLCPP_WARN(this->get_logger(), "Point is too far. Aborting");
      return;
    }
  }

  tf2::Quaternion tf2_quat(pose.pose.orientation.x, pose.pose.orientation.y,
                           pose.pose.orientation.z, pose.pose.orientation.w);
  tf2_quat = this->initial_transform * tf2_quat;
  this->temp_orientation = Eigen::Quaterniond(tf2_quat.w(), tf2_quat.x(),
                                              tf2_quat.y(), tf2_quat.z());
  Eigen::Quaterniond quat(tf2_quat.w(), tf2_quat.x(), -tf2_quat.y(),
                          -tf2_quat.z());

  setpoint_msg.position = {
      transformed.x(),
      -transformed.y(),
      -transformed.z(),
  };
  setpoint_msg.yaw =
      px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(
          quat);
  setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  this->trajectory_setpoint_pub_->publish(setpoint_msg);
}

void Controller::publish_vehicle_command(uint16_t command, float param1,
                                         float param2) {
  px4_msgs::msg::VehicleCommand command_msg{};
  command_msg.command = command;
  command_msg.param1 = param1;
  command_msg.param2 = param2;
  command_msg.target_system = 1;
  command_msg.target_component = 1;
  command_msg.source_system = 1;
  command_msg.source_component = 1;
  command_msg.from_external = true;
  command_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  this->vehicle_command_pub_->publish(command_msg);
}

void Controller::publish_offboard_control_mode() {
  px4_msgs::msg::OffboardControlMode offboard_msg{};
  offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_msg.position = true;
  offboard_msg.velocity = false;
  offboard_msg.acceleration = false;
  offboard_msg.attitude = false;
  offboard_msg.body_rate = false;

  this->offboard_mode_pub_->publish(offboard_msg);
}

void Controller::arm() {
  publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0,
      0.0);
  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void Controller::disarm() {
  publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0,
      0.0);
  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void Controller::goal_response_callback(ReplanGoalHandle::SharedPtr future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void Controller::feedback_callback(
    ReplanGoalHandle::SharedPtr,
    const std::shared_ptr<const ReplanAction::Feedback> feedback) {
  std::stringstream ss;
  ss << "Received: ";
  ss << feedback->state << " ";
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void Controller::result_callback(
    const ReplanGoalHandle::WrappedResult &result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  if (this->path_sequence == result.result->path_sequence) {
    RCLCPP_INFO(this->get_logger(), "Path is the same");
  }
  this->path_sequence = result.result->path_sequence;
  nav_msgs::msg::Path::SharedPtr path =
      std::make_shared<nav_msgs::msg::Path>(result.result->path);
  this->path_cb(path);
}

}  // namespace SimplePlanner

int main(int argc, char **argv) {
  Core::BaseArgumentParser parser(argc, argv);

  parser.add_argument("--goal-tolerance")
      .default_value(0.5)
      .help(
          "tolerance to determine whether the drone reached the goal (meters)")
      .scan<'f', float>();
  parser.add_argument("--max-distance")
      .default_value(20)
      .help(
          "max distance the drone will try to move to reach the goal (meters)")
      .scan<'f', float>();
  parser.add_argument("--enforce-max-distance")
      .help("whether the controller will force the value from distance")
      .flag()
      .implicit_value(true);
  parser.add_argument("--min-height")
      .default_value(1.5)
      .help("minimum distance the drone will be above the ground level")
      .scan<'f', float>();
  parser.add_argument("--reached-subscription-topic")
      .default_value("reached_position")
      .help(
          "topic where the notification when a goal was reached will be sent "
          "to");
  parser.add_argument("--smooth-weight-data")
      .default_value(0.5)
      .scan<'f', float>();
  parser.add_argument("--smooth-weight-smooth")
      .default_value(0.1)
      .scan<'f', float>();
  parser.add_argument("--smooth-tolerance")
      .default_value(1.0)
      .scan<'f', float>();
  parser.add_argument("--step-size").default_value(1.0).scan<'f', float>();
  parser.add_argument("--autorequest")
      .implicit_value(true)
      .help(
          "whether the drone will autorequest a new plan in case the goal was "
          "not reached")
      .flag();
  parser.add_argument("--yaw-tolerance").default_value(0.3).scan<'f', float>();

  std::shared_ptr<SimplePlanner::Controller> controller =
      std::make_shared<SimplePlanner::Controller>(parser);
  controller->run();
  return 0;
}

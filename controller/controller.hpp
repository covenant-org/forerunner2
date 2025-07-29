#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/planner.capnp.h>
#include <chrono>
#include <cstdint>

using namespace std::chrono_literals;

namespace SimplePlanner {
using ReplanAction = planner_interfaces::action::Replan;
using ReplanGoalHandle = rclcpp_action::ClientGoalHandle<ReplanAction>;

class Controller : public Core::Vertex {
 public:
  Controller(Core::ArgumentParser parser);
  ~Controller() {};
  void path_cb(const Core::IncomingMessage<Path> &path);
  void local_pose_cb(const Core::IncomingMessage<PoseStamped> &pose);
  void control();
  void arm();
  void disarm();
  void smooth_path();

 private:
  std::shared_ptr<Core::Publisher<PoseStamped>> _reacher_goal_pub;
  std::shared_ptr<Core::Subscriber<HomePosition>> _home_pos_sub;
  std::shared_ptr<Core::Subscriber<Path>> _path_sub;
  std::shared_ptr<Core::Subscriber<Telemetry>> _telemetry_pub;
  std::shared_ptr<Core::ActionClient<Command, GenericResponse>> _action_client;
  std::shared_ptr<Core::Publisher<Path>> _smoothed_path_pub;
  std::shared_ptr<Core::Publisher<Path>> _trajectory_path_pub;

  Path::Reader _path;
  bool recived_path;
  bool waiting_for_path;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
      vehicle_command_pub_;
  nav_msgs::msg::Path path;
  rclcpp::TimerBase::SharedPtr timer_;
  px4_msgs::msg::VehicleLocalPosition vehicle_initial_position;
  Eigen::Vector3d current_position;
  Eigen::Vector3d last_path_start_position;
  bool recorded_initial_position;
  int index;
  int sent_point;
  char debauncer;
  bool recived_path;
  bool waiting_for_path;
  Eigen::Vector3d temp_goal;
  Eigen::Vector3d home_position;
  Eigen::Quaterniond temp_orientation;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2::Transform initial_transform;
  bool waiting_reponse;
  size_t path_sequence;
  std::shared_ptr<Core::ActionClient<ReplanRequest, ReplanResponse>>
      _action_client;
  void goal_response_callback(ReplanGoalHandle::SharedPtr future);
  void feedback_callback(
      ReplanGoalHandle::SharedPtr handle,
      const std::shared_ptr<const ReplanAction::Feedback> feedback);
  void result_callback(const ReplanGoalHandle::WrappedResult &result);
  rclcpp_action::Client<ReplanAction>::SendGoalOptions goal_options;
};
}  // namespace SimplePlanner
   //
#endif

#include "controller.hpp"
#include "message.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <capnp_schemas/generics.capnp.h>
#include <chrono>
#include <functional>
#include <thread>

Controller::Controller(Core::ArgumentParser parser) : Core::Vertex(parser) {
  this->_reached_goal_pub = this->create_publisher<PoseStamped>("reached_goal");
  this->_smoothed_path_pub = this->create_publisher<Path>("smoothed_path");
  this->_home_pos_sub = this->create_subscriber<HomePosition>(
      "home_position", [this](const Core::IncomingMessage<HomePosition> &msg) {
        this->home_position(0) = msg.content.getPos().getX();
        this->home_position(1) = msg.content.getPos().getY();
        this->home_position(2) = msg.content.getPos().getZ();
      });
  this->_path_sub = this->create_subscriber<Path>(
      "planned_path",
      std::bind(&Controller::planned_path_cb, this, std::placeholders::_1));
  this->_odometry_sub = this->create_subscriber<Odometry>(
      "odometry",
      std::bind(&Controller::odometry_cb, this, std::placeholders::_1));
  this->_telemetry_sub = this->create_subscriber<Telemetry>(
      "telemetry",
      std::bind(&Controller::telemetry_cb, this, std::placeholders::_1));
  this->_controller_client =
      this->create_action_client<Command, GenericResponse>("controller");
  this->_planner_client =
      this->create_action_client<ReplanRequest, GenericResponse>("planner");

  this->_control_thread = std::thread([&] {
    while (true) {
      this->control();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  this->index = 0;
  this->sent_point = 0;
  this->recived_path = false;
  this->recorded_initial_position = false;
  this->_vehicle_initial_position.x() = 0.0;
  this->_vehicle_initial_position.y() = 0.0;
  this->_vehicle_initial_position.z() = 0.0;
  this->debauncer = 0;
}

void Controller::odometry_cb(const Core::IncomingMessage<Odometry> &msg) {
  auto o = msg.content;
  auto q = msg.content.getQ();
  auto pos = msg.content.getPosition();
  _heading = msg.content.getHeading();
  _quat = Eigen::Quaternionf(q.getW(), q.getX(), q.getY(), q.getZ());
  _position = Eigen::Vector3f(pos.getX(), pos.getY(), pos.getZ());

  this->current_position = Eigen::Vector3f(
      o.getPosition().getX(), o.getPosition().getY(), o.getPosition().getZ());

  if (!this->waiting_reponse && this->recived_path &&
      this->index < this->_path.getPoses().size()) {
    Eigen::Vector3d current_position(
        o.getPosition().getX(), o.getPosition().getY(), o.getPosition().getZ());
    Eigen::Quaterniond current_orientation(o.getQ().getW(), o.getQ().getX(),
                                           o.getQ().getY(), o.getQ().getZ());

    double dist = std::sqrt((_position - this->temp_goal).squaredNorm());
    double tolerance = this->get_argument<double>("--goal-tolerance");
    this->_logger.debug("Distance to goal: %f, tolerance: %f", dist, tolerance);
    double angular_dist =
        current_orientation.angularDistance(this->temp_orientation);
    this->_logger.debug("Orientation to goal: %f", angular_dist);

    bool autorequest = this->get_argument<bool>("--autorequest");
    double yaw_tolerance = this->get_argument<double>("--yaw-tolerance");

    if (autorequest && dist < tolerance && angular_dist < yaw_tolerance) {
      auto msg = this->_planner_client->new_msg();
      auto start = msg.content.initStart();
      start.setCurrentPathIndex(this->index);
      start.initPose();
      auto pose = start.getPose();
      pose.initPose();
      pose.getPose().initPosition();
      pose.getPose().getPosition().setX(o.getPosition().getX());
      pose.getPose().getPosition().setY(o.getPosition().getY());
      pose.getPose().getPosition().setZ(o.getPosition().getZ());

      this->waiting_reponse = true;
      auto res = msg.send();
    }
  }

  if (this->recorded_initial_position) {
    return;
  }

  this->_vehicle_initial_position.x() = o.getPosition().getX();
  this->_vehicle_initial_position.y() = o.getPosition().getY();
  this->_vehicle_initial_position.z() = o.getPosition().getZ();
  this->recorded_initial_position = true;
  this->_logger.info("recorded initial position");
}

void Controller::telemetry_cb(const Core::IncomingMessage<Telemetry> &msg) {}

void Controller::smooth_path() {
  const double weight_data = this->get_argument<double>("--smooth-weight-data");
  const double weight_smooth =
      this->get_argument<double>("--smooth-weight-smooth");
  const double tolerance = this->get_argument<double>("--smooth-tolerance");

  auto msg = this->_smoothed_path_pub->new_msg();
  auto original_poses = this->_path.getPoses();
  auto poses = msg.content.initPoses(this->_path.getPoses().size());
  double change = tolerance;
  this->_logger.debug("smoothing path");
  while (change >= tolerance) {
    change = 0.0;
    for (size_t i = 1; i < this->_path.getPoses().size() - 1; i++) {
      Eigen::Vector3d original_position(
          this->_path.getPoses()[i].getPose().getPosition().getX(),
          this->_path.getPoses()[i].getPose().getPosition().getY(),
          this->_path.getPoses()[i].getPose().getPosition().getZ());
      Eigen::Vector3d current_position(
          original_poses[i].getPose().getPosition().getX(),
          original_poses[i].getPose().getPosition().getY(),
          original_poses[i].getPose().getPosition().getZ());
      Eigen::Vector3d next_position(
          original_poses[i + 1].getPose().getPosition().getX(),
          original_poses[i + 1].getPose().getPosition().getY(),
          original_poses[i + 1].getPose().getPosition().getZ());
      Eigen::Vector3d prev_position(
          original_poses[i - 1].getPose().getPosition().getX(),
          original_poses[i - 1].getPose().getPosition().getY(),
          original_poses[i - 1].getPose().getPosition().getZ());
      Eigen::Vector3d smoothed =
          original_position +
          weight_data * (original_position - current_position) +
          weight_smooth *
              (next_position + prev_position - 2.0 * current_position);
      change += Eigen::abs((original_position - smoothed).array()).sum();
      this->_logger.debug("Change: %f, original: %f %f %f smoothed: %f %f %f",
                          change, original_position(0), original_position(1),
                          original_position(2), smoothed(0), smoothed(1),
                          smoothed(2));
      auto pose = poses[i].initPose();
      auto position = poses[i].getPose().initPosition();
      position.setX(smoothed(0));
      position.setY(smoothed(1));
      position.setZ(smoothed(2));
    }
    this->_logger.debug("Change: %f", change);
  }
  this->_logger.debug("Path smoothed");
  msg.publish();
  this->_path = msg.content.asReader();
}

void Controller::planned_path_cb(const Core::IncomingMessage<Path> &msg) {
  auto path = msg.content;
  this->waiting_reponse = false;
  if (path.getPoses().size() == 0) return;
  auto poses = path.getPoses();
  auto first_pose = poses[0];
  Eigen::Vector3d first_coord(first_pose.getPose().getPosition().getX(),
                              first_pose.getPose().getPosition().getY(),
                              first_pose.getPose().getPosition().getZ());
  // auto trajectory_msg = this->_trajectory_path_pub->new_msg();
  // trajectory_msg.content.setPoses(poses);
  // trajectory_msg.publish();
  this->index = std::min(1, int(path.getPoses().size() - 1));

  auto pose = this->_path.getPoses()[this->index];
  this->_last_path_start_position =
      Eigen::Vector3f(first_coord(0), first_coord(1), first_coord(2));
  this->recived_path = true;
}

void Controller::control() {
  if (!this->recived_path) {
    return;
  }

  if (sent_point == 10) {
    this->arm();
  }

  // this->publish_offboard_control_mode();

  if (sent_point < 10) {
    auto msg = this->_controller_client->new_msg();
    auto wp = msg.content.initWaypoint();
    wp.setX(this->_vehicle_initial_position.x());
    wp.setY(-this->_vehicle_initial_position.y());
    wp.setZ(-this->_vehicle_initial_position.z());
    wp.setR(this->_heading * M_PI / 180);
    auto result = msg.send();
    auto response = result.value().content;
    if (response.getCode() != 200) {
      this->_logger.error(
          "Trajectory setpoint command failed - Code: %d, Message: %s",
          response.getCode(), response.getMessage().cStr());
    }
  } else {
    // TODO: Check if path is not empty and pose is valid
    auto pose = _path.getPoses()[this->index];
    this->publish_trajectory_setpoint(pose);
  }
  this->sent_point += 1;

  this->_logger.debug("publish point");
}

void Controller::publish_trajectory_setpoint(PoseStamped::Reader &pose) {
  Eigen::Vector3f point(pose.getPose().getPosition().getX(),
                        pose.getPose().getPosition().getY(),
                        pose.getPose().getPosition().getZ());

  Eigen::Vector3f transformed = this->_initial_transform * point;
  double min_height = this->get_argument<double>("--min-height");
  if (transformed.z() < min_height) {
    transformed.z() = min_height;
  }

  double step_size = this->get_argument<double>("--step-size");
  Eigen::Vector3f ePoint(transformed.x(), transformed.y(), transformed.z());

  double distance_to_point = (ePoint - this->_last_path_start_position).norm();
  this->_logger.debug("Distance to point: %f", distance_to_point);
  if (distance_to_point > step_size) {
    this->_logger.debug(
        "Current point %f, %f, %f", this->_last_path_start_position(0),
        this->_last_path_start_position(1), this->_last_path_start_position(2));
    this->_logger.debug("Original point %f, %f, %f", ePoint(0), ePoint(1),
                        ePoint(2));
    double percentaje = step_size / distance_to_point;
    transformed = Eigen::Vector3f(
        (this->_last_path_start_position(0) +
         (ePoint(0) - this->_last_path_start_position(0)) * percentaje),
        (this->_last_path_start_position(1) +
         (ePoint(1) - this->_last_path_start_position(1)) * percentaje),
        (this->_last_path_start_position(2) +
         (ePoint(2) - this->_last_path_start_position(2)) * percentaje));
    this->_logger.debug("Modified %f, %f, %f", transformed.x(), transformed.y(),
                        transformed.z());
  }

  this->temp_goal =
      Eigen::Vector3f(transformed.x(), transformed.y(), transformed.z());
  bool enforce_max_distance =
      this->get_argument<bool>("--enforce-max-distance");
  if (enforce_max_distance) {
    double max_distance = this->get_argument<double>("--max-distance");
    ePoint = Eigen::Vector3f(transformed.x(), transformed.y(), transformed.z());
    double distance = (ePoint - this->home_position).norm();
    if (distance > max_distance) {
      this->_logger.debug("Point is too far. Aborting");
      return;
    }
  }
  auto orientation = pose.getPose().getOrientation();

  auto tf_quat = Eigen::Quaternionf(orientation.getW(), orientation.getX(),
                                    orientation.getY(), orientation.getZ());
  tf_quat = this->_quat * tf_quat;
  this->temp_orientation =
      Eigen::Quaterniond(tf_quat.w(), tf_quat.x(), tf_quat.y(), tf_quat.z());
  Eigen::Quaterniond quat(tf_quat.w(), tf_quat.x(), -tf_quat.y(), -tf_quat.z());

  auto msg = this->_controller_client->new_msg();
  msg.content.initWaypoint();
  auto wp = msg.content.getWaypoint();
  wp.setX(transformed.x());
  wp.setY(-transformed.y());
  wp.setZ(-transformed.z());
  wp.setR(this->_heading * M_PI / 180);
  auto res = msg.send();
  auto response = res.value().content;
  if (response.getCode() != 200) {
    this->_logger.error(
        "Trajectory setpoint command failed - Code: %d, Message: %s",
        response.getCode(), response.getMessage().cStr());
  }
}

// void Controller::publish_vehicle_command(uint16_t command, float param1,
//                                          float param2) {
//   px4_msgs::msg::VehicleCommand command_msg{};
//   command_msg.command = command;
//   command_msg.param1 = param1;
//   command_msg.param2 = param2;
//   command_msg.target_system = 1;
//   command_msg.target_component = 1;
//   command_msg.source_system = 1;
//   command_msg.source_component = 1;
//   command_msg.from_external = true;
//   command_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//
//   this->vehicle_command_pub_->publish(command_msg);
// }

// void Controller::publish_offboard_control_mode() {
//   px4_msgs::msg::OffboardControlMode offboard_msg{};
//   offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//   offboard_msg.position = true;
//   offboard_msg.velocity = false;
//   offboard_msg.acceleration = false;
//   offboard_msg.attitude = false;
//   offboard_msg.body_rate = false;
//
//   this->offboard_mode_pub_->publish(offboard_msg);
// }

void Controller::arm() {}

void Controller::disarm() {}

// void Controller::goal_response_callback(ReplanGoalHandle::SharedPtr future) {
//   auto goal_handle = future.get();
//   if (!goal_handle) {
//     RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//   } else {
//     RCLCPP_INFO(this->get_logger(),
//                 "Goal accepted by server, waiting for result");
//   }
// }

// void Controller::feedback_callback(
//     ReplanGoalHandle::SharedPtr,
//     const std::shared_ptr<const ReplanAction::Feedback> feedback) {
//   std::stringstream ss;
//   ss << "Received: ";
//   ss << feedback->state << " ";
//   RCLCPP_INFO(this->get_logger(), ss.str().c_str());
// }

// void Controller::result_callback(
//     const ReplanGoalHandle::WrappedResult &result) {
//   switch (result.code) {
//     case rclcpp_action::ResultCode::SUCCEEDED:
//       break;
//     case rclcpp_action::ResultCode::ABORTED:
//       RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//       return;
//     case rclcpp_action::ResultCode::CANCELED:
//       RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//       return;
//     default:
//       RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//       return;
//   }
//   if (this->path_sequence == result.result->path_sequence) {
//     RCLCPP_INFO(this->get_logger(), "Path is the same");
//   }
//   this->path_sequence = result.result->path_sequence;
//   nav_msgs::msg::Path::SharedPtr path =
//       std::make_shared<nav_msgs::msg::Path>(result.result->path);
//   this->path_cb(path);
// }

void Controller::run() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser parser(argc, argv);

  parser.add_argument("--goal-tolerance")
      .default_value(0.5)
      .help(
          "tolerance to determine whether the drone reached the goal (meters)")
      .scan<'g', float>();
  parser.add_argument("--max-distance")
      .default_value(20)
      .help(
          "max distance the drone will try to move to reach the goal (meters)")
      .scan<'g', float>();
  parser.add_argument("--enforce-max-distance")
      .help("whether the controller will force the value from distance")
      .flag()
      .implicit_value(true);
  parser.add_argument("--min-height")
      .default_value(1.5)
      .help("minimum distance the drone will be above the ground level")
      .scan<'g', float>();
  parser.add_argument("--reached-subscription-topic")
      .default_value("reached_position")
      .help(
          "topic where the notification when a goal was reached will be sent "
          "to");
  parser.add_argument("--smooth-weight-data")
      .default_value(0.5)
      .scan<'g', float>();
  parser.add_argument("--smooth-weight-smooth")
      .default_value(0.1)
      .scan<'g', float>();
  parser.add_argument("--smooth-tolerance")
      .default_value(1.0)
      .scan<'g', float>();
  parser.add_argument("--step-size").default_value(1.0).scan<'g', float>();
  parser.add_argument("--autorequest")
      .implicit_value(true)
      .help(
          "whether the drone will autorequest a new plan in case the goal was "
          "not reached")
      .flag();
  parser.add_argument("--yaw-tolerance").default_value(0.3).scan<'g', float>();

  std::shared_ptr<Controller> controller = std::make_shared<Controller>(parser);
  controller->run();
  return 0;
}

#include "argument_parser.hpp"
#include "forerunner2_ros_bridge/controller.hpp"
#include "message.hpp"
#include <capnp_schemas/generics.capnp.h>
#include <cstdio>
#include <iostream>
#include <memory>

using namespace std::chrono;
using namespace std::chrono_literals;

RosController::RosController(Core::ArgumentParser args)
    : Core::Vertex(args), rclcpp::Node("RosController") {
  this->_logger.info("Constructor");
  this->_odom_pub = this->Core::Vertex::create_publisher<Odometry>("odometry");
  this->_tel_pub = this->Core::Vertex::create_publisher<Telemetry>("telemetry");
  this->_server =
      this->Core::Vertex::create_action_server<Command, GenericResponse>(
          "controller",
          std::bind(&RosController::handle_commands, this,
                    std::placeholders::_1, std::placeholders::_2));

  _offboard_control_mode_publisher =
      this->rclcpp::Node::create_publisher<px4_msgs::msg::OffboardControlMode>(
          "/fmu/in/offboard_control_mode", 10);
  _actuators_publisher =
      this->rclcpp::Node::create_publisher<px4_msgs::msg::ActuatorMotors>(
          "/fmu/in/actuator_motors", 10);
  _vehicle_command_publisher =
      this->rclcpp::Node::create_publisher<px4_msgs::msg::VehicleCommand>(
          "/fmu/in/vehicle_command", 10);
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                         qos_profile);

  this->_odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos,
      [this](px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
        auto output = this->_odom_pub->new_msg();
        auto angular = output.content.initAngular();
        auto pos = output.content.initPosition();
        auto vel = output.content.initVelocity();
        auto q = output.content.initQ();
        angular.setX(msg->angular_velocity[0]);
        angular.setZ(-msg->angular_velocity[2]);
        angular.setY(-msg->angular_velocity[1]);
        pos.setX(msg->position[0]);
        pos.setY(-msg->position[1]);
        pos.setZ(-msg->position[2]);
        vel.setX(msg->velocity[0]);
        vel.setY(-msg->velocity[1]);
        vel.setZ(-msg->velocity[2]);
        q.setW(msg->q[0]);
        q.setX(msg->q[1]);
        q.setY(msg->q[2]);
        q.setZ(msg->q[3]);
        output.publish();
      });
  this->_tel_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos,
      [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
        auto output = this->_tel_pub->new_msg();
        output.content.setArmed(msg->arming_state == 2);
        output.publish();
      });
}

void RosController::publish_vehicle_command(uint16_t command, float param1,
                                            float param2, float param3) {
  px4_msgs::msg::VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _vehicle_command_publisher->publish(msg);
}

void RosController::arm() {
  publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}
void RosController::disarm() {
  publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void RosController::publish_control() {
  px4_msgs::msg::ActuatorMotors msg{};
  this->_logger.info("==Actuators==");
  for (int i = 0; i < _actuators.size(); i++) {
    this->_logger.info("%f", _actuators[i]);
    msg.control[i] = _actuators[i];
  }
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _actuators_publisher->publish(msg);
}
void RosController::publish_offboard_control_mode() {
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.direct_actuator = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _offboard_control_mode_publisher->publish(msg);
}

void RosController::handle_commands(
    const Core::IncomingMessage<Command> &command,
    GenericResponse::Builder &res) {
  this->_logger.debug("Requested command");
  res.setCode(200);
  res.setMessage("OK");
  switch (command.content.which()) {
    case Command::LAND: {
      this->publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0,
          6.0);
      return;
    }
    case Command::TAKEOFF: {
      //      if (!this->_telemetry->armed()) {
      //        const auto arm_result = this->_action->arm();
      //        if (arm_result != mavsdk::Action::Result::Success) {
      //          res.setCode(500);
      //          res.setMessage("Failed to arm");
      //          return;
      //        }
      //      }
      this->publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0,
          2.0);
      return;
    }
    case Command::OFFBOARD: {
      this->_offboard_setpoint_counter = 0;
      auto timer_callback = [this]() -> void {
        if (_offboard_setpoint_counter == 10) {
          // Change to Offboard mode after 10 setpoints
          this->publish_vehicle_command(
              px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

          // Arm the vehicle
          this->arm();
        }

        // offboard_control_mode needs to be paired with trajectory_setpoint
        publish_offboard_control_mode();
        publish_control();

        // stop the counter after reaching 11
        if (_offboard_setpoint_counter < 11) {
          _offboard_setpoint_counter++;
        }
      };
      _timer = this->create_wall_timer(100ms, timer_callback);

      /*
if (!command.content.getOffboard().getEnable()) {
const auto offboard_result = this->_offboard->stop();
if (offboard_result != mavsdk::Offboard::Result::Success) {
res.setCode(500);
res.setMessage("Error while stopping offboard");
}
return;
}

// Check prerequisites for offboard mode
if (!this->_telemetry->armed()) {
res.setCode(400);
res.setMessage("Cannot start offboard: drone not armed");
return;
}

// Log current flight mode
auto current_mode = this->_telemetry->flight_mode();
this->_logger.debug("Current flight mode before offboard: %d",
  static_cast<int>(current_mode));

const auto offboard_result = this->_offboard->start();
if (offboard_result != mavsdk::Offboard::Result::Success) {
std::string error_msg = "Error starting offboard: ";
switch (offboard_result) {
case mavsdk::Offboard::Result::NoSystem:
error_msg += "No system connected";
break;
case mavsdk::Offboard::Result::ConnectionError:
error_msg += "Connection error";
break;
case mavsdk::Offboard::Result::Busy:
error_msg += "System busy";
break;
case mavsdk::Offboard::Result::CommandDenied:
error_msg += "Command denied by autopilot";
break;
case mavsdk::Offboard::Result::Timeout:
error_msg += "Timeout";
break;
case mavsdk::Offboard::Result::NoSetpointSet:
error_msg += "No setpoint set - send waypoint first";
break;
default:
error_msg += "Unknown error";
break;
}
res.setCode(500);
res.setMessage(error_msg);
return;
}
*/

      this->_logger.info("Offboard mode started successfully");
      return;
    }
    case Command::ARM: {
      this->_logger.debug("Requested arm");
      //      if (this->_telemetry->armed()) {
      //        res.setCode(304);
      //        res.setMessage("Already armed");
      //        return;
      //      }

      //     const auto arm_result = this->_action->arm();
      // if (arm_result != mavsdk::Action::Result::Success) {
      //   res.setCode(500);
      //   res.setMessage("Failed to arm");
      // return;
      // }
      this->arm();
      this->_logger.info("Armed succesfully");
      return;
    }
    case Command::WAYPOINT: {
      /*
this->_logger.debug("Entered Command::WAYPOINT");
auto waypoint = command.content.getWaypoint();
this->_logger.debug("WAYPOINT coords: x=%f y=%f z=%f r=%f",
  waypoint.getX(), waypoint.getY(), waypoint.getZ(),
  waypoint.getR());

// Check if armed - allow waypoints for armed drones even if not in air
// yet
if (!this->_telemetry->armed()) {
this->_logger.error("Drone must be armed for waypoint commands");
res.setCode(400);
res.setMessage("Drone not armed - cannot set waypoints");
return;
}

// Set the position setpoint - offboard mode must be started separately by
// client
const auto set_res =
this->_offboard->set_position_ned({.north_m = waypoint.getX(),
                     .east_m = waypoint.getY(),
                     .down_m = waypoint.getZ(),
                     .yaw_deg = waypoint.getR()});
if (set_res != mavsdk::Offboard::Result::Success) {
std::string error_msg = "set_position_ned failed: ";
switch (set_res) {
case mavsdk::Offboard::Result::NoSystem:
error_msg += "No system connected";
break;
case mavsdk::Offboard::Result::ConnectionError:
error_msg += "Connection error";
break;
default:
error_msg +=
"Error code " + std::to_string(static_cast<int>(set_res));
break;
}
this->_logger.error("%s", error_msg.c_str());
res.setCode(500);
res.setMessage("Failed to set position setpoint");
return;
}
*/

      this->_logger.debug(
          "Waypoint setpoint set successfully - ready for offboard mode");
      return;
    }
    case Command::SET_ACTUATORS: {
      auto actuators = command.content.getSetActuators();
      this->_actuators.resize(actuators.size());
      for (int i = 0; i < actuators.size(); i++) {
        this->_actuators[i] = actuators[i];
      }
      return;
    }
    case Command::SET_ATTITUDE: {
      /*
auto attitude = command.content.getSetAttitude();
mavsdk::Offboard::Attitude ctl;
auto q = attitude.getQ();
ctl.q[0] = q[0];
ctl.q[1] = q[1];
ctl.q[2] = q[2];
ctl.q[3] = q[3];
ctl.thrust_value = attitude.getThrust();
auto ctl_res = this->_offboard->set_attitude(ctl);
if (ctl_res != mavsdk::Offboard::Result::Success) {
this->_logger.error("Error sending actuators control");
res.setCode(500);
res.setMessage("Error sending actuators control");
}
*/
      return;
    }
    default:
      res.setCode(501);
      res.setMessage("Not implemented");
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  Core::BaseArgumentParser args(argc, argv);
  rclcpp::spin(std::make_shared<RosController>(args));
  rclcpp::shutdown();
  return 0;
}

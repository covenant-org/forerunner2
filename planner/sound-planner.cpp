#include "argument_parser.hpp"
#include "message.hpp"
#include "sound-planner.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unistd.h>
#include <utility>

#define M_3_PI_2 4.7123889803846896740 /* 3/pi/2 */

SoundPlanner::SoundPlanner(Core::ArgumentParser args)
    : Core::Vertex(args), quart(1, 0, 0, 0), position(0, 0, 0), goal(0, 0, 0) {
  this->_odmetry_sub = this->create_subscriber<Odometry>(
      "odometry",
      std::bind(&SoundPlanner::odometry_cb, this, std::placeholders::_1));
  this->_telemetry_sub = this->create_subscriber<Telemetry>(
      "telemetry",
      std::bind(&SoundPlanner::telemetry_cb, this, std::placeholders::_1));
  this->_command_client =
      this->create_action_client<Command, GenericResponse>("controller");
  this->_mic_sub = this->create_subscriber<StereoMic>(
      "mic", std::bind(&SoundPlanner::mic_cb, this, std::placeholders::_1));
  this->_altitude_sub = this->create_subscriber<Altitude>(
      "altitude",
      std::bind(&SoundPlanner::altitude_cb, this, std::placeholders::_1));

  this->target_altitude = args.get_argument<float>("altitude");
  if (target_altitude <= 0 || !std::isfinite(target_altitude)) {
    target_altitude = 2;
  }
  if (target_altitude < 2) {
    this->_logger.warn("Target altitude is less than 2m proceed with caution");
  }
  this->_telemetry_count = 0;
  this->_is_armed = false;
  this->_is_in_air = false;
  this->_arm_requested = false;
  this->_takeoff_requested = false;
  this->_lmic = 0;
  this->_rmic = 0;
  this->altitude = 0;
}

void SoundPlanner::altitude_cb(const Core::IncomingMessage<Altitude> &msg) {
  altitude = msg.content.getAvg();
  this->_logger.debug("Received altitude msg: %.3f", altitude);
}

void SoundPlanner::mic_cb(const Core::IncomingMessage<StereoMic> &msg) {
  this->_logger.debug("Received odometry message");
  auto mic = msg.content;
  this->_lmic = mic.getLeft();
  this->_rmic = mic.getRight();
  this->_logger.debug("Mic: %.2f  %.2f", this->_lmic, this->_rmic);
}

void SoundPlanner::odometry_cb(const Core::IncomingMessage<Odometry> &msg) {
  this->_logger.debug("Received odometry message");
  auto odom = msg.content;
  auto pos = odom.getPosition();
  auto q = odom.getQ();
  quart = Eigen::Quaternionf(q.getW(), q.getX(), q.getY(), q.getZ());
  position = Eigen::Vector3f(pos.getX(), pos.getY(), pos.getZ());
  this->_logger.debug("Pos xyz: %.2f \t %.2f \t %.2f", position.x(),
                      position.y(), position.z());
}

void SoundPlanner::telemetry_cb(const Core::IncomingMessage<Telemetry> &msg) {
  this->_logger.debug("Received telemetry message");
  auto telemetry = msg.content;
  this->_logger.debug("Battery: %f", telemetry.getBattery().getPercentage());
  this->_logger.debug("Mode: %s", telemetry.getMode().cStr());
  this->_logger.debug("In Air: %d", telemetry.getInAir());
  this->_logger.debug("Armed: %d", telemetry.getArmed());

  this->_telemetry_count++;
  this->_is_armed = telemetry.getArmed();
  this->_is_in_air = telemetry.getInAir();
}

float SoundPlanner::calc_mic_diff() {
  float diff = std::abs(_rmic - _lmic);
  float avg = (_rmic + _lmic) / 2;
  return diff / avg;
}

SoundPlanner::Waypoint SoundPlanner::next_waypoint(const int &forward_m = 20) {
  Eigen::Matrix2f rotation;
  float diff = _lmic - _rmic;
  auto yaw = quart.toRotationMatrix().eulerAngles(0, 1, 2)[2];
  this->_logger.info("dif: %f, yaw: %f", diff, yaw * 180 / M_PI);
  yaw += diff > 0 ? -M_PI_2 : M_PI_2;
  if (yaw > 2 * M_PI) {
    yaw -= 2 * M_PI;
  }
  if (yaw < 0) {
    yaw += 2 * M_PI;
  }
  rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  Eigen::Vector2f movement(forward_m * std::abs(diff), 0);
  movement = rotation * movement;
  if (yaw > M_PI) {
    yaw -= 2 * M_PI;
  }
  return {.point = Eigen::Vector3f(position.x() + movement.x(),
                                   position.y() + movement.y(), position.z()),
          .yaw_deg = static_cast<float>(yaw * 180 / M_PI)};
}

void SoundPlanner::run() {
  this->_logger.info("Running");
  this->_logger.debug("Waiting for 5 telemetry messages");
  while (this->_telemetry_count < 5) {
    usleep(200000);
  }
  this->_logger.debug("Ready to proceed");
  while (!this->_is_in_air) {
    if (!this->_is_armed && !this->_arm_requested) {
      this->_logger.info("Drone not armed, sending arm command");
      auto request = this->_command_client->new_msg();
      request.content.setArm();

      auto result = request.send();
      auto response = result.value().content;
      if (response.getCode() >= 300) {
        this->_logger.error("Arm command failed - Code: %d, Message: %s",
                            response.getCode(), response.getMessage().cStr());
        return;
      }
      this->_logger.info("Arm command sent successfully");
      this->_arm_requested = true;
      usleep(200000);
      continue;
    }

    if (this->_is_armed && !this->_is_in_air && !this->_takeoff_requested) {
      this->_logger.info("Drone armed but not in air, sending takeoff command");
      auto request = this->_command_client->new_msg();
      auto takeoff = request.content.initTakeoff();
      takeoff.setAltitude(2);

      auto result = request.send();
      auto response = result.value().content;
      if (response.getCode() >= 300) {
        this->_logger.error("Takeoff command failed - Code: %d, Message: %s",
                            response.getCode(), response.getMessage().cStr());
        return;
      }
      this->_logger.info("Takeoff command sent successfully");
      this->_takeoff_requested = true;
      sleep(5);
      continue;
    }

    usleep(200000);
  }
  while (altitude < target_altitude && (target_altitude - altitude) > 0.5) {
    this->_logger.debug("Waiting for drone to reach altitude");
    sleep(1);
  }
  this->_logger.info("Drone is now in the air");
  auto point_msg = this->_command_client->new_msg();
  auto point = point_msg.content.initWaypoint();
  point.setX(position.x());
  point.setY(position.y());
  point.setZ(position.z());
  goal = position;
  point_msg.send();
  auto offboard_msg = this->_command_client->new_msg();
  auto off = offboard_msg.content.initOffboard();
  off.setEnable(true);
  offboard_msg.send();
  float diff = this->calc_mic_diff();
  while (diff > 0.01) {
    diff = this->calc_mic_diff();
    while ((goal - position).norm() > 0.3) {
      this->_logger.debug("Waiting for drone to reach goal");
      sleep(1);
      continue;
    }
    this->_logger.debug("Diff %f", diff);
    auto waypoint = next_waypoint();
    goal = waypoint.point;
    this->_logger.info(
        "Current position: %.2f, %.2f, %.2f. Target position: %.2f, %.2f, "
        "%.2f. Target yaw: %.2f",
        position.x(), position.y(), position.z(), goal.x(), goal.y(), goal.z(),
        waypoint.yaw_deg);
    auto goal_msg = this->_command_client->new_msg();
    auto point = goal_msg.content.initWaypoint();
    point.setX(goal.x());
    point.setY(goal.y());
    point.setZ(goal.z());
    point.setR(waypoint.yaw_deg);
    goal_msg.send();
    getchar();
    sleep(1);
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser args(argc, argv);
  args.add_argument("--altitude")
      .default_value(2.0f)
      .nargs(1)
      .help("Target altitude in meters");
  SoundPlanner planner(args);
  planner.run();
}

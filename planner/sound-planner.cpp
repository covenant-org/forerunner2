#include "argument_parser.hpp"
#include "message.hpp"
#include "sound-planner.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <unistd.h>

const Eigen::Quaternionf ROT_90DEG(std::cos(M_PI_4), 0, 0, std::sin(M_PI_4));

SoundPlanner::SoundPlanner(Core::ArgumentParser args) : Core::Vertex(args) {
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

  this->_telemetry_count = 0;
  this->_is_armed = false;
  this->_is_in_air = false;
  this->_arm_requested = false;
  this->_takeoff_requested = false;
  this->_lmic = 0;
  this->_rmic = 0;
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
  position = Eigen::Quaternionf(0, pos.getX(), pos.getY(), pos.getZ());
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
      usleep(200000);
      continue;
    }

    usleep(200000);
  }
  this->_logger.info("Drone is now in the air");
  auto point_msg = this->_command_client->new_msg();
  auto point = point_msg.content.initWaypoint();
  point.setX(position.x());
  point.setY(position.y());
  point.setZ(position.z());
  point_msg.send();
  auto offboard_msg = this->_command_client->new_msg();
  auto off = offboard_msg.content.initOffboard();
  off.setEnable(true);
  offboard_msg.send();
  float diff = this->calc_mic_diff();
  while (diff > 0.01) {
    this->_logger.debug("Diff %f", diff);
    auto msg = this->_command_client->new_msg();
    auto point = msg.content.initWaypoint();
    auto rotation = ROT_90DEG * quart;
    Eigen::Quaternionf translation(0, 0, 3, 0);
    auto transrot = rotation * translation * rotation.conjugate();
    point.setX(position.x() + transrot.x());
    point.setY(position.y() + transrot.y());
    point.setZ(position.z() + transrot.z());
    msg.send();
    sleep(1);
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser args(argc, argv);
  SoundPlanner planner(args);
  planner.run();
}

#include "sound-planner.hpp"
#include <unistd.h>

SoundPlanner::SoundPlanner(int argc, char **argv) : Core::Vertex(argc, argv) {
  this->_odmetry_sub = this->create_subscriber<Odometry>(
      "odometry",
      std::bind(&SoundPlanner::odometry_cb, this, std::placeholders::_1));
  this->_telemetry_sub = this->create_subscriber<Telemetry>(
      "telemetry",
      std::bind(&SoundPlanner::telemetry_cb, this, std::placeholders::_1));
  this->_command_client =
      this->create_action_client<Command, GenericResponse>("controller");

  this->_telemetry_count = 0;
  this->_is_armed = false;
  this->_is_in_air = false;
  this->_arm_requested = false;
  this->_takeoff_requested = false;
}

void SoundPlanner::odometry_cb(const Core::IncomingMessage<Odometry> &msg) {
  this->_logger.debug("Received odometry message");
  auto odom = msg.content;
  auto pos = odom.getPosition();
  this->_logger.debug("Pos xyz: %.2f \t %.2f \t %.2f", pos.getX(), pos.getY(),
                      pos.getZ());
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

void SoundPlanner::run() {
  this->_logger.info("Running");
  while (!this->_is_in_air) {
    if (this->_telemetry_count < 5) {
      usleep(200000);
      continue;
    }

    if (this->_is_armed && this->_is_in_air) {
      break;
    }

    if (!this->_is_armed && !this->_arm_requested) {
      this->_logger.info("Drone not armed, sending arm command");
      auto request = this->_command_client->new_msg();
      request.content.setArm();

      auto result = request.send();
      if (result.has_value()) {
        auto response = result.value().content;
        if (response.getCode() >= 300) {
          this->_logger.error("Arm command failed - Code: %d, Message: %s",
                              response.getCode(), response.getMessage().cStr());
        } else {
          this->_arm_requested = true;
          this->_logger.info("Arm command sent successfully");
        }
      } else {
        this->_logger.error("Failed to receive response for arm command");
      }
      usleep(200000);
      continue;
    }

    if (this->_is_armed && !this->_is_in_air && !this->_takeoff_requested) {
      this->_logger.info("Drone armed but not in air, sending takeoff command");
      auto request = this->_command_client->new_msg();
      auto takeoff = request.content.initTakeoff();
      takeoff.setAltitude(5);

      auto result = request.send();
      if (result.has_value()) {
        auto response = result.value().content;
        if (response.getCode() >= 300) {
          this->_logger.error("Takeoff command failed - Code: %d, Message: %s",
                              response.getCode(), response.getMessage().cStr());
        } else {
          this->_takeoff_requested = true;
          this->_logger.info("Takeoff command sent successfully");
        }
      } else {
        this->_logger.error("Failed to receive response for takeoff command");
      }
    }

    usleep(200000);
  }
  this->_logger.info("Drone is now in the air");
}

int main(int argc, char **argv) {
  SoundPlanner planner(argc, argv);
  planner.run();
}

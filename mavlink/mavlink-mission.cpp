#include "mavlink-mission.hpp"
#include "message.hpp"
#include <capnp_schemas/mavlink.capnp.h>
#include <chrono>
#include <thread>

MavlinkMission::MavlinkMission(Core::ArgumentParser parser)
    : Core::Vertex(parser) {
  _command_client =
      this->create_action_client<Command, GenericResponse>("controller");
  _mission_server = this->create_action_server<MissionCommand, GenericResponse>(
      "mission_command",
      std::bind(&MavlinkMission::mission_command_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  _alt_sub = this->create_subscriber<Altitude>(
      "altitude",
      std::bind(&MavlinkMission::altitude_cb, this, std::placeholders::_1));
  _tel_sub = this->create_subscriber<Telemetry>(
      "telemetry",
      std::bind(&MavlinkMission::telemetry_cb, this, std::placeholders::_1));
}

void MavlinkMission::mission_command_cb(
    const Core::IncomingMessage<MissionCommand> &mission,
    GenericResponse::Builder &res) {
  switch (mission.content.which()) {
    case MissionCommand::TAKEOFF: {
      auto takeoff = mission.content.getTakeoff();
      auto desired_altitude = takeoff.getDesiredAltitude();
      this->arm_and_takeoff(res, desired_altitude);
    } break;
    case MissionCommand::LAND: {
      this->land(res);
    } break;
  }
}

void MavlinkMission::telemetry_cb(const Core::IncomingMessage<Telemetry> &msg) {
  auto t = msg.content;
  this->_is_armed = t.getArmed();
  this->_is_in_air = t.getInAir();
}

void MavlinkMission::altitude_cb(const Core::IncomingMessage<Altitude> &msg) {
  this->_altitude = msg.content.getAvg();
}

void MavlinkMission::arm_and_takeoff(GenericResponse::Builder &res,
                                     float desired_altitude) {
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
        res.setCode(response.getCode());
        res.setMessage(res.getMessage());
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
      takeoff.setAltitude(desired_altitude);

      auto result = request.send();
      auto response = result.value().content;
      if (response.getCode() >= 300) {
        this->_logger.error("Takeoff command failed - Code: %d, Message: %s",
                            response.getCode(), response.getMessage().cStr());
        res.setCode(response.getCode());
        res.setMessage(res.getMessage());
        return;
      }
      this->_logger.info("Takeoff command sent successfully");
      this->_takeoff_requested = true;
      this->_land_requested = false;
      sleep(5);
      continue;
    }

    usleep(200000);
  }
  while (_altitude < desired_altitude && (desired_altitude - _altitude) > 0.5) {
    this->_logger.debug("Waiting for drone to reach altitude");
    sleep(1);
  }
  this->_logger.info("Drone is now in the air");
  res.setCode(200);
  res.setMessage("ok");
}

void MavlinkMission::land(GenericResponse::Builder &res) {
  while (_is_in_air) {
    if (_is_armed && !this->_land_requested) {
      _arm_requested = false;
      _takeoff_requested = false;

      auto request = this->_command_client->new_msg();
      request.content.setLand();
      auto result = request.send();
      auto response = result.value().content;
      if (response.getCode() >= 300) {
        this->_logger.error("Land command failed - Code: %d, Message: %s",
                            response.getCode(), response.getMessage().cStr());
        res.setCode(response.getCode());
        res.setMessage(res.getMessage());
        return;
      }
      this->_land_requested = true;
    }
  }
  this->_logger.info("Drone has now landed");
  res.setCode(200);
  res.setMessage("ok");
}

void MavlinkMission::run() {
  while (true) std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser parser(argc, argv);
  std::shared_ptr<MavlinkMission> mission =
      std::make_shared<MavlinkMission>(parser);
  mission->run();
  return 0;
}

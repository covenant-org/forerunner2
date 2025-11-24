#include "argument_parser.hpp"
#include "message.hpp"
#include "publisher.hpp"
#include "utils.hpp"
#include "vertex.hpp"
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/nav_msgs.capnp.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

class FakeMavlink : public Core::Vertex {
 private:
  std::shared_ptr<Core::Publisher<HomePosition>> _home_position_publisher;
  std::shared_ptr<Core::Publisher<Odometry>> _odometry_publisher;
  std::shared_ptr<Core::Publisher<Telemetry>> _telemetry_publisher;
  std::shared_ptr<Core::Publisher<Altitude>> _altitude_publisher;
  std::shared_ptr<Core::Publisher<KeyValue>> _config_publisher;
  std::shared_ptr<Core::ActionServer<Command, GenericResponse>>
      _command_action_server;

  bool armed;
  bool in_air;
  float pos[3];
  float q[4];
  std::string mode;
  bool init_mavlink_connection(const std::string &);

  struct State {
    float bat;
    bool arm;
    bool inar;
    std::string mode;
  } _telemetry_state;
  float heading;

 public:
  FakeMavlink(Core::ArgumentParser);
  void command_cb(const Core::IncomingMessage<Command> &,
                  GenericResponse::Builder &);
  void publish_odometry();
  void publish_telemtry();

  void run();
};

FakeMavlink::FakeMavlink(Core::ArgumentParser parser)
    : Core::Vertex(std::move(parser)) {
  _command_action_server = create_action_server<Command, GenericResponse>(
      "controller", std::bind(&FakeMavlink::command_cb, this,
                              std::placeholders::_1, std::placeholders::_2));
  this->_home_position_publisher =
      this->create_publisher<HomePosition>("home_position");
  this->_odometry_publisher = this->create_publisher<Odometry>("odometry");
  this->_telemetry_publisher = this->create_publisher<Telemetry>("telemetry");
  this->_altitude_publisher = this->create_publisher<Altitude>("altitude");
  q[0] = 0;
  q[1] = 0;
  q[2] = 0;
  q[3] = 0;
  pos[0] = this->get_argument<float>("-x");
  pos[1] = this->get_argument<float>("-y");
  pos[2] = 0;
}

void FakeMavlink::command_cb(const Core::IncomingMessage<Command> &command,
                             GenericResponse::Builder &res) {
  this->_logger.debug("Requested command");
  res.setCode(200);
  res.setMessage("OK");
  switch (command.content.which()) {
    case Command::LAND: {
      sleep(1);
      this->pos[2] = 0;
      this->armed = false;
      this->in_air = false;
      return;
    }
    case Command::TAKEOFF: {
      auto info = command.content.getTakeoff();
      sleep(1);
      this->armed = true;
      this->in_air = true;
      this->pos[2] = -info.getAltitude();
      return;
    }
    case Command::OFFBOARD: {
      // Check prerequisites for offboard mode
      if (!this->armed) {
        res.setCode(400);
        res.setMessage("Cannot start offboard: drone not armed");
        return;
      }

      if (!this->in_air) {
        res.setCode(400);
        res.setMessage("Cannot start offboard: drone not in air");
        return;
      }
      this->mode = "offboard";
      this->_logger.info("Offboard mode started successfully");
      return;
    }
    case Command::ARM: {
      this->_logger.debug("Requested arm");
      if (armed) {
        res.setCode(304);
        res.setMessage("Already armed");
        return;
      }
      this->armed = true;

      this->_logger.info("Armed succesfully");
      return;
    }
    case Command::WAYPOINT: {
      this->_logger.debug("Entered Command::WAYPOINT");
      auto waypoint = command.content.getWaypoint();
      auto waypoint_pos = waypoint.getPosition();
      this->_logger.debug("WAYPOINT coords: x=%f y=%f z=%f r=%f",
                          waypoint_pos.getX(), waypoint_pos.getY(), waypoint_pos.getZ(),
                          waypoint.getYaw());
      pos[0] = waypoint_pos.getX();
      pos[1] = waypoint_pos.getY();
      pos[2] = waypoint_pos.getZ();

      // Check if armed - allow waypoints for armed drones even if not in air
      // yet
      if (!this->armed) {
        this->_logger.error("Drone must be armed for waypoint commands");
        res.setCode(400);
        res.setMessage("Drone not armed - cannot set waypoints");
        return;
      }
      sleep(5);

      this->_logger.debug(
          "Waypoint setpoint set successfully - ready for offboard mode");
      return;
    }
    case Command::GOTO_LOCATION: {
      // Check if armed - goto_location requires armed drone
      if (!this->armed) {
        this->_logger.error("Drone must be armed for GPS waypoint commands");
        res.setCode(400);
        res.setMessage("Drone not armed - cannot goto GPS location");
        return;
      }
      sleep(1);

      this->_logger.info("GPS waypoint command sent successfully");
      return;
    }
    case Command::UPLOAD_MISSION: {
      this->_logger.debug("Entered Command::UPLOAD_MISSION");
      sleep(1);
      // Create MissionPlan and upload
      this->_logger.info("Mission uploaded successfully");
      return;
    }
    case Command::START_MISSION: {
      this->_logger.debug("Entered Command::START_MISSION");

      if (!this->armed) {
        res.setCode(400);
        res.setMessage("Cannot start mission: drone not armed");
        return;
      }

      this->_logger.info("Mission started successfully");
      return;
    }
    case Command::PAUSE_MISSION: {
      this->_logger.debug("Entered Command::PAUSE_MISSION");

      this->_logger.info("Mission paused successfully");
      return;
    }
    case Command::CLEAR_MISSION: {
      this->_logger.debug("Entered Command::CLEAR_MISSION");

      this->_logger.info("Mission cleared successfully");
      return;
    }
    default:
      res.setCode(501);
      res.setMessage("Not implemented");
  }
}

bool FakeMavlink::init_mavlink_connection(const std::string &uri) {
  return true;
}

void FakeMavlink::publish_telemtry() {
  auto msg = this->_telemetry_publisher->new_msg();
  msg.content.setArmed(this->armed);
  auto battery = msg.content.initBattery();
  battery.setPercentage(80);
  msg.content.setInAir(this->in_air);
  msg.content.setMode(this->mode);
  msg.publish();
}

void FakeMavlink::publish_odometry() {
  auto msg = this->_odometry_publisher->new_msg();
  auto pose_w_cov = msg.content.initPose();
  auto pose = pose_w_cov.initPose();
  auto twist_w_cov = msg.content.initTwist();
  auto twist = twist_w_cov.initTwist();
  auto angular = twist.initAngular();
  auto pos = pose.initPosition();
  auto vel = twist.initLinear();
  auto q = pose.initOrientation();
  angular.setX(0);
  angular.setZ(0);
  angular.setY(0);
  pos.setX(this->pos[0]);
  pos.setY(this->pos[1]);
  pos.setZ(this->pos[2]);
  vel.setX(0);
  vel.setY(1);
  vel.setZ(2);
  q.setX(0);
  q.setY(0);
  q.setZ(0);
  q.setW(1);
  msg.publish();
}

void FakeMavlink::run() {
  Core::RateKeeper rk(1);
  while (true) {
    publish_telemtry();
    publish_odometry();
    auto altitude_msg = this->_altitude_publisher->new_msg();
    altitude_msg.content.setLocal(-this->pos[2]);
    altitude_msg.content.setAvg(-this->pos[2]);
    altitude_msg.content.setRelative(-this->pos[2]);
    altitude_msg.publish();
    this->_logger.info("Position: %f %f %f", pos[0], pos[1], pos[2]);
    rk.keep();
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser base(argc, argv);
  base.add_argument("--mavlink-uri")
      .help("ip where the mavlink instance is running")
      .nargs(1)
      .default_value("");
  base.add_argument("--ftp-dir")
      .help("Root directory for the ftp server")
      .default_value("/tmp/mavlink.XXXXXX");
  base.add_argument("-x").default_value(0).scan<'g', float>().help(
      "Initial x position");
  base.add_argument("-y").default_value(0).scan<'g', float>().help(
      "Initial y position");

  std::shared_ptr<FakeMavlink> mavlink =
      std::make_shared<FakeMavlink>(std::move(base));
  mavlink->run();
  return 0;
}

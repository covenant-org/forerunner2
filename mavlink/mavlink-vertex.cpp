#include "argument_parser.hpp"
#include "mavlink-vertex.hpp"
#include "message.hpp"
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <mavsdk/connection_result.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <plugins/mavlink_passthrough/mavlink_passthrough.h>

MavlinkVertex::MavlinkVertex(Core::ArgumentParser parser)
    : Core::Vertex(std::move(parser)),
      _mavsdk(
          mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}) {
  auto uri = this->get_argument<std::string>("--mavlink-uri");
  auto result = this->init_mavlink_connection(uri);
  if (!result) {
    throw std::runtime_error("error initializing mavsdk");
  }

  _command_action_server = create_action_server<Command, GenericResponse>(
      "controller", std::bind(&MavlinkVertex::command_cb, this,
                              std::placeholders::_1, std::placeholders::_2));
  this->_home_position_publisher =
      this->create_publisher<HomePosition>("home_position");
  this->_odometry_publisher = this->create_publisher<Odometry>("odometry");
  this->_telemetry_publisher = this->create_publisher<Telemetry>("telemetry");
  this->_altitude_publisher = this->create_publisher<Altitude>("altitude");
}

void MavlinkVertex::command_cb(const Core::IncomingMessage<Command> &command,
                               GenericResponse::Builder &res) {
  this->_logger.debug("Requested command");
  res.setCode(200);
  res.setMessage("OK");
  switch (command.content.which()) {
    case Command::LAND: {
      const auto land_result = this->_action->land();
      if (land_result != mavsdk::Action::Result::Success) {
        res.setCode(500);
        res.setMessage("Error while landing");
      }
      return;
    }
    case Command::TAKEOFF: {
      if (!this->_telemetry->armed()) {
        const auto arm_result = this->_action->arm();
        if (arm_result != mavsdk::Action::Result::Success) {
          res.setCode(500);
          res.setMessage("Failed to arm");
          return;
        }
      }
      const auto set_takeoff_altitude_result =
          this->_action->set_takeoff_altitude(
              command.content.getTakeoff().getAltitude());
      if (set_takeoff_altitude_result != mavsdk::Action::Result::Success) {
        res.setCode(500);
        res.setMessage("Failed to set takeoff altitude");
        return;
      }
      const auto takeoff_result = this->_action->takeoff();
      if (takeoff_result != mavsdk::Action::Result::Success) {
        res.setCode(500);
        res.setMessage("Failed to takeoff");
      }
      return;
    }
    case Command::OFFBOARD: {
      if (!command.content.getOffboard().getEnable()) {
        const auto offboard_result = this->_offboard->stop();
        if (offboard_result != mavsdk::Offboard::Result::Success) {
          res.setCode(500);
          res.setMessage("Error while stoping offboard");
        }
        return;
      }
      const auto offboard_result = this->_offboard->start();
      if (offboard_result != mavsdk::Offboard::Result::Success) {
        res.setCode(500);
        res.setMessage("Error while enabiling offboard");
      }
      return;
    }
    case Command::ARM: {
      this->_logger.debug("Requested arm");
      if (this->_telemetry->armed()) {
        res.setCode(304);
        res.setMessage("Already armed");
        return;
      }

      const auto arm_result = this->_action->arm();
      if (arm_result != mavsdk::Action::Result::Success) {
        res.setCode(500);
        res.setMessage("Failed to arm");
        return;
      }
      this->_logger.info("Armed succesfully");
      return;
    }
    case Command::WAYPOINT: {
      auto waypoint = command.content.getWaypoint();
      const auto waypoint_res =
          this->_offboard->set_position_ned({.north_m = waypoint.getX(),
                                             .east_m = waypoint.getY(),
                                             .down_m = waypoint.getZ(),
                                             .yaw_deg = waypoint.getR()});
      if (waypoint_res != mavsdk::Offboard::Result::Success) {
        res.setCode(500);
        res.setMessage("Failed to set position");
        return;
      }
      return;
    }
    default:
      res.setCode(501);
      res.setMessage("Not implemented");
  }
}

bool MavlinkVertex::init_mavlink_connection(const std::string &uri) {
  mavsdk::ConnectionResult result = this->_mavsdk.add_any_connection(uri);
  if (result != mavsdk::ConnectionResult::Success) {
    return false;
  }

  this->_system = this->_mavsdk.first_autopilot(3.0);
  if (!this->_system) {
    return false;
  }

  this->_telemetry = std::make_shared<mavsdk::Telemetry>(this->_system.value());
  this->_action = std::make_shared<mavsdk::Action>(this->_system.value());
  this->_offboard = std::make_shared<mavsdk::Offboard>(this->_system.value());
  this->_passthrough =
      std::make_shared<mavsdk::MavlinkPassthrough>(this->_system.value());

  return true;
}

void MavlinkVertex::publish_telemtry() {
  auto msg = this->_telemetry_publisher->new_msg();
  msg.content.setArmed(this->_telemetry_state.arm);
  auto battery = msg.content.initBattery();
  battery.setPercentage(this->_telemetry_state.bat);
  msg.content.setInAir(this->_telemetry_state.inar);
  msg.content.setMode(this->_telemetry_state.mode);
  msg.publish();
}

void MavlinkVertex::odometry_cb(const mavsdk::Telemetry::Odometry &odom) {
  auto msg = this->_odometry_publisher->new_msg();
  auto angular = msg.content.initAngular();
  auto pos = msg.content.initPosition();
  auto vel = msg.content.initVelocity();
  auto q = msg.content.initQ();
  angular.setX(odom.angular_velocity_body.roll_rad_s);
  angular.setZ(odom.angular_velocity_body.pitch_rad_s);
  angular.setY(odom.angular_velocity_body.yaw_rad_s);
  pos.setX(odom.position_body.x_m);
  pos.setY(odom.position_body.y_m);
  pos.setZ(odom.position_body.z_m);
  vel.setX(odom.velocity_body.x_m_s);
  vel.setY(odom.velocity_body.y_m_s);
  vel.setZ(odom.velocity_body.z_m_s);
  q.setX(odom.q.x);
  q.setY(odom.q.y);
  q.setZ(odom.q.z);
  q.setW(odom.q.w);
  msg.content.setHeading(this->heading);
  msg.publish();
}

void MavlinkVertex::run() {
  this->_passthrough->subscribe_message(
      HOME_POSITION_MESSAGE_ID, [this](const __mavlink_message &msg) {
        mavlink_msg_home_position_decode(&msg, &this->_mavlink_home_position);
        auto home_position_msg = this->_home_position_publisher->new_msg();
        home_position_msg.content.getPos().setX(this->_mavlink_home_position.x);
        home_position_msg.content.getPos().setY(
            -this->_mavlink_home_position.y);
        home_position_msg.content.getPos().setZ(
            -this->_mavlink_home_position.z);
        home_position_msg.publish();
      });

  this->_telemetry->subscribe_odometry(
      std::bind(&MavlinkVertex::odometry_cb, this, std::placeholders::_1));

  this->_telemetry->subscribe_battery(
      [this](const mavsdk::Telemetry::Battery &bat) {
        this->_telemetry_state.bat = bat.remaining_percent;
        this->publish_telemtry();
      });

  this->_telemetry->subscribe_armed([this](const bool &arm) {
    this->_telemetry_state.arm = arm;
    this->publish_telemtry();
    this->_logger.debug("Armed %d", arm);
  });

  this->_telemetry->subscribe_in_air([this](const bool &inar) {
    this->_telemetry_state.inar = inar;
    this->publish_telemtry();
  });

  this->_telemetry->subscribe_flight_mode(
      [this](const mavsdk::Telemetry::FlightMode &mode) {
        std::stringstream ss;
        ss << mode;
        this->_telemetry_state.mode = ss.str();
        this->publish_telemtry();
      });

  this->_telemetry->subscribe_altitude(
      [this](const mavsdk::Telemetry::Altitude &altitude) {
        auto msg = this->_altitude_publisher->new_msg();
        auto alt = msg.content;
        alt.setLocal(altitude.altitude_local_m);
        alt.setRelative(altitude.altitude_relative_m);
        alt.setMonotonic(altitude.altitude_monotonic_m);
        alt.setAvg(
            (altitude.altitude_monotonic_m + altitude.altitude_relative_m) / 2);
        msg.publish();
      });

  this->_telemetry->subscribe_heading(
      [this](const mavsdk::Telemetry::Heading &heading) {
        this->heading = heading.heading_deg;
      });

  while (true) {
    sleep(1);
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser base(argc, argv);
  base.add_argument("--mavlink-uri")
      .help("ip where the mavlink instance is running")
      .nargs(1)
      .default_value(MAVLINK_URI);

  std::shared_ptr<MavlinkVertex> mavlink =
      std::make_shared<MavlinkVertex>(std::move(base));
  mavlink->run();
  return 0;
}

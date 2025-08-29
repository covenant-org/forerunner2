#include "argument_parser.hpp"
#include "commander.hpp"
#include <sstream>
#include <vector>
#include <array>
#include <cmath>
#include <mutex>

// Simple conversion lat/lon/alt -> NED relative to home 
static void latlon_to_ned_simple(double lat, double lon, double alt,
                                 double home_lat, double home_lon, double home_alt,
                                 double &north_m, double &east_m, double &down_m) {
  const double R = 6378137.0; // WGS84 equatorial radius (m)
  double dLat = (lat - home_lat) * M_PI / 180.0;
  double dLon = (lon - home_lon) * M_PI / 180.0;
  double lat_avg = (lat + home_lat) * M_PI / 360.0;
  north_m = dLat * R;
  east_m = dLon * R * std::cos(lat_avg);
  // down is positive downward; alt & home_alt are "up" positive -> down = -(alt - home_alt)
  down_m = -(alt - home_alt);
}

Commander::Commander(Core::ArgumentParser parser) : Core::Vertex(parser) {
  this->_mission_client =
      this->create_action_client<MissionCommand, GenericResponse>("mission_command");
  this->_controller_client =
      this->create_action_client<Command, GenericResponse>("controller");

  // Subscriber for home_position published by Mavlink
  this->_home_subscriber = this->create_subscriber<HomePosition>(
      "home_position",
      [this](const Core::IncomingMessage<HomePosition> &msg) {
        auto GPS = msg.content.getGps();
        std::lock_guard<std::mutex> lk(this->_home_mutex);
        // Only accept the first home_position message
        if (this->_home.set) {
          return;
        }
        this->_home.set = true;
        this->_home.pos.lat = GPS.getLatitude() / 1e7;
        this->_home.pos.lon = GPS.getLongitude() / 1e7;
        this->_home.pos.alt = GPS.getAltitude() / 1000.0;
        this->_logger.debug("Home received: Lat=%f Lon=%f Alt=%f", this->_home.pos.lat, this->_home.pos.lon, this->_home.pos.alt);
      });
}

void Commander::run() {
  while (true) {
    std::string line;
    std::cout << "> " << std::flush;
    
    if (!std::getline(std::cin, line)) {
      break; // EOF or error
    }
    
    std::istringstream iss(line);
    std::string command;
    iss >> command;

    std::vector<std::string> args;
    std::string token;
    while (iss >> token) {
      args.push_back(token);
    }

    this->_logger.debug("Command: %s", command.c_str());

    if (command == "takeoff") {
      // Usage: takeoff [altitude]
      float desired_alt = 2.0f;
      if (!args.empty()) {
        try {
          desired_alt = std::stof(args[0]);
        } catch (...) {
          this->_logger.warn("Invalid altitude argument, using default 2");
        }
      }
      auto request = this->_mission_client->new_msg();
      request.content.initTakeoff();
      request.content.getTakeoff().setDesiredAltitude(desired_alt);
      auto result = request.send();
      auto response = result.value().content;
      if (response.getCode() != 200) {
        this->_logger.error("Takeoff failed with code %d and message %s",
                            response.getCode(), response.getMessage());
      }
    } else if (command == "land") {
      auto request = this->_mission_client->new_msg();
      request.content.setLand();
      auto result = request.send();
      auto response = result.value().content;
      if (response.getCode() != 200) {
        this->_logger.error("Land failed with code %d and message %s",
                            response.getCode(), response.getMessage());
      }
    } else if (command == "waypoint") {
          if (!args.empty() && args.size() > 1) {
            // Rebuild the remainder of the input after 'local' or 'global'
            std::string coords_str;
            for (size_t i = 1; i < args.size(); ++i) {
              if (!coords_str.empty()) coords_str += ' ';
              coords_str += args[i];
            }
            // Normalize commas to spaces so we can parse with >>
            for (char &c : coords_str) {
              if (c == ',') c = ' ';
            }

            std::istringstream css(coords_str);
            std::array<double, 4> coords = {0.0, 0.0, 0.0, 0.0};
            double value;
            size_t idx = 0;
            while (idx < coords.size() && (css >> value)) {
              coords[idx++] = value;
            }
            
            this->_logger.debug("Coords: %f, %f, %f, %f",
                                coords[0], coords[1],
                                coords[2], coords[3]);

            if (args[0] == "local") {
              // usage: waypoint local <x> <y> <z> [<yaw>]
              // Support '~' to keep previous values
              double x = this->_last_local.x;
              double y = this->_last_local.y;
              double z = this->_last_local.z;
              double yaw = this->_last_local.yaw;
              if (args.size() > 1 && args[1] != "~") x = coords[0];
              if (args.size() > 2 && args[2] != "~") y = coords[1];
              if (args.size() > 3 && args[3] != "~") z = coords[2];
              if (args.size() > 4 && args[4] != "~") yaw = coords[3];

              // Update last used values
              this->_last_local.x = x;
              this->_last_local.y = y;
              this->_last_local.z = z;
              this->_last_local.yaw = yaw;

              auto cmd_req = this->_controller_client->new_msg();
              auto wp = cmd_req.content.initWaypoint();
              wp.setX(static_cast<float>(x));
              wp.setY(static_cast<float>(y));
              wp.setZ(-static_cast<float>(z));
              wp.setR(static_cast<float>(yaw));

              auto cmd_res = cmd_req.send();
              auto resp = cmd_res.value().content;
              if (resp.getCode() != 200) {
                this->_logger.error("Controller refused waypoint: %s", resp.getMessage());
              }
            }
            else if (args[0] == "global") {
              // usage: waypoint global <lat> <lon> <alt> [<yaw>]
              // Support '~' to keep previous values
              if (!this->_home.set) {
                this->_logger.error("Home not set.");
              } else {
                double lat = this->_last_global.pos.lat;
                double lon = this->_last_global.pos.lon;
                double alt = this->_last_global.pos.alt;
                double yaw = this->_last_global.yaw;
                if (args.size() > 1 && args[1] != "~") lat = coords[0];
                if (args.size() > 2 && args[2] != "~") lon = coords[1];
                if (args.size() > 3 && args[3] != "~") alt = coords[2];
                if (args.size() > 4 && args[4] != "~") yaw = coords[3];

                // Update last used values
                this->_last_global.pos.lat = lat;
                this->_last_global.pos.lon = lon;
                this->_last_global.pos.alt = alt;
                this->_last_global.yaw = yaw;

                double north, east, down;
                latlon_to_ned_simple(lat, lon, alt, this->_home.pos.lat, this->_home.pos.lon, this->_home.pos.alt, north, east, down);

                // Forward as local waypoint
                auto cmd_req = this->_controller_client->new_msg();
                auto wp = cmd_req.content.initWaypoint();
                wp.setX(static_cast<float>(north));
                wp.setY(static_cast<float>(east));
                wp.setZ(static_cast<float>(down));
                wp.setR(static_cast<float>(yaw));

                auto cmd_res = cmd_req.send();
                auto resp = cmd_res.value().content;
                if (resp.getCode() != 200) {
                  this->_logger.error("Controller refused waypoint: %s", resp.getMessage());
                } else {
                  this->_logger.debug("Global -> NED: north=%f east=%f down=%f yaw=%f", north, east, down, yaw);
                }
              }
            }
          } else {
            
          }
    } else {
      this->_logger.warn("Unknown command: %s", command.c_str());
    }
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser parser(argc, argv);
  std::shared_ptr<Commander> commander = std::make_shared<Commander>(parser);
  commander->run();
  return 0;
}

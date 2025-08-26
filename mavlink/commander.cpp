#include "argument_parser.hpp"
#include "commander.hpp"
#include <sstream>
#include <vector>
#include <array>

Commander::Commander(Core::ArgumentParser parser) : Core::Vertex(parser) {
  this->_mission_client =
      this->create_action_client<MissionCommand, GenericResponse>("mission_command");
    this->_controller_client =
        this->create_action_client<Command, GenericResponse>("controller");
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
      
      auto request = this->_mission_client->new_msg();
      request.content.initTakeoff();
      request.content.getTakeoff().setDesiredAltitude(2);
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
          // Example: "waypoint local 19,20,-21,0" or "waypoint local 19 20 -21 0"
          if (!args.empty() && args.size() > 1) {
            // Rebuild the remainder of the input after 'local'
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
              // build and send a Command::WAYPOINT to the controller action server
              auto cmd_req = this->_controller_client->new_msg();
              auto wp = cmd_req.content.initWaypoint();
              wp.setX(static_cast<float>(coords[0]));
              wp.setY(static_cast<float>(coords[1]));
              wp.setZ(static_cast<float>(coords[2]));
              wp.setR(static_cast<float>(coords[3]));

              auto cmd_res = cmd_req.send();
              auto resp = cmd_res.value().content;
              if (resp.getCode() != 200) {
                this->_logger.error("Controller refused waypoint: %s", resp.getMessage());
              } else {
                this->_logger.info("Waypoint forwarded to controller");
              }
            }
            else if (args[0] == "global") {
              // TO DO: Agregar transformacion de global a local
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

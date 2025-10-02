#include "argument_parser.hpp"
#include "commander.hpp"
#include <argparse/argparse.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>

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
                this->_logger.error("Controller refused waypoint: Code=%d, Message=%s", 
                                    resp.getCode(), resp.getMessage().cStr());
              }
            }
            else if (args[0] == "global") {
              // usage: waypoint global <lat> <lon> <alt> [<yaw>]
              // Support '~' to keep previous values
              double lat = this->_last_global.pos.lat;
              double lon = this->_last_global.pos.lon;
              double alt = this->_last_global.pos.alt;
              double yaw = this->_last_global.yaw;
              if (args.size() > 1 && args[1] != "~") lat = coords[0];
              if (args.size() > 2 && args[2] != "~") lon = coords[1];
              if (args.size() > 3 && args[3] != "~") alt = coords[2];
              if (args.size() > 4 && args[4] != "~") yaw = coords[3];

              this->_last_global.pos.lat = lat;
              this->_last_global.pos.lon = lon;
              this->_last_global.pos.alt = alt;
              this->_last_global.yaw = yaw;

              auto cmd_req = this->_controller_client->new_msg();
              auto gps_wp = cmd_req.content.initGotoLocation();
              gps_wp.setLatitude(lat);
              gps_wp.setLongitude(lon);
              gps_wp.setAltitude(static_cast<float>(alt));
              gps_wp.setYaw(static_cast<float>(yaw));

              auto cmd_res = cmd_req.send();
              auto resp = cmd_res.value().content;
              if (resp.getCode() != 200) {
                this->_logger.error("Controller refused GPS waypoint: Code=%d, Message=%s", 
                                    resp.getCode(), resp.getMessage().cStr());
              } else {
                this->_logger.debug("GPS waypoint sent: lat=%f lon=%f alt=%f yaw=%f", lat, lon, alt, yaw);
              }
            }
          } else {
            this->_logger.warn("Usage: waypoint [local|global] <coords>");
          }
    } else if (command == "offboard") {
      // Usage: offboard [on|off|enable|disable|true|false|1|0]
      bool enable = true; // default to enable
      if (!args.empty()) {
        std::string arg = args[0];
        std::transform(arg.begin(), arg.end(), arg.begin(), ::tolower);
        if (arg == "off" || arg == "disable" || arg == "false" || arg == "0") {
          enable = false;
        } else if (arg == "on" || arg == "enable" || arg == "true" || arg == "1") {
          enable = true;
        } else {
          this->_logger.warn("Invalid offboard argument '%s', using default 'on'", args[0].c_str());
        }
      }
      
      auto cmd_req = this->_controller_client->new_msg();
      auto offboard_cmd = cmd_req.content.initOffboard();
      offboard_cmd.setEnable(enable);
      
      auto cmd_res = cmd_req.send();
      auto resp = cmd_res.value().content;
      if (resp.getCode() != 200) {
        this->_logger.error("Offboard command failed: %s", resp.getMessage().cStr());
      } else {
        this->_logger.info("Offboard mode %s", enable ? "enabled" : "disabled");
      }
    } else if (command == "mission") {
      this->handle_mission_command(args);
    } else if (command == "help" || command == "?") {
      std::cout << "\nAvailable commands:" << std::endl;
      std::cout << "  takeoff [altitude]          - Takeoff to specified altitude (default: 2m)" << std::endl;
      std::cout << "  land                        - Land the drone" << std::endl;
      std::cout << "  waypoint local <x> <y> <z> [yaw] - Go to local position" << std::endl;
      std::cout << "  waypoint global <lat> <lon> <alt> [yaw] - Go to GPS position" << std::endl;
      std::cout << "  offboard [on|off]           - Enable/disable offboard mode" << std::endl;
      std::cout << "  mission upload <file.yaml>   - Upload mission from YAML file" << std::endl;
      std::cout << "  mission start               - Start uploaded mission" << std::endl;
      std::cout << "  mission pause               - Pause running mission" << std::endl;
      std::cout << "  mission clear               - Clear current mission" << std::endl;
      std::cout << "  help, ?                     - Show this help" << std::endl;
      std::cout << "  exit, quit                  - Exit interactive mode" << std::endl;
      std::cout << "\nNote: Use '~' to keep previous values in waypoint commands" << std::endl;
    } else if (command == "exit" || command == "quit") {
      std::cout << "Goodbye!" << std::endl;
      break;
    } else {
      this->_logger.warn("Unknown command: %s", command.c_str());
      std::cout << "Type 'help' for available commands." << std::endl;
    }
  }
}

void Commander::handle_mission_command(const std::vector<std::string>& args) {
  if (args.size() < 1) {
    this->_logger.warn("Usage: mission <upload|start|pause|clear> [args...]");
    return;
  }
  
  std::string mission_cmd = args[0];
  
  if (mission_cmd == "upload") {
    if (args.size() < 2) {
      this->_logger.warn("Usage: mission upload <waypoint_file.yaml>");
      return;
    }
    
    std::string filename = args[1];
    auto mission_items = this->load_mission_from_file(filename);
    if (mission_items.empty()) {
      this->_logger.error("Failed to load mission from file: %s", filename.c_str());
      return;
    }
    
    auto cmd_req = this->_controller_client->new_msg();
    auto upload = cmd_req.content.initUploadMission();
    auto waypoints = upload.initWaypoints(mission_items.size());
    
    for (size_t i = 0; i < mission_items.size(); i++) {
      auto wp = waypoints[i];
      wp.setLatitude(mission_items[i].latitude);
      wp.setLongitude(mission_items[i].longitude);
      wp.setRelativeAltitude(mission_items[i].altitude);
      wp.setSpeed(mission_items[i].speed);
      wp.setIsFlyThrough(mission_items[i].is_fly_through);
      wp.setGimbalPitch(mission_items[i].gimbal_pitch);
      wp.setGimbalYaw(mission_items[i].gimbal_yaw);
      wp.setLoiterTime(mission_items[i].loiter_time);
      wp.setCameraPhotoInterval(mission_items[i].camera_photo_interval);
      
      if (mission_items[i].camera_action == "TAKE_PHOTO") {
        wp.setCameraAction(MissionItem::CameraAction::TAKE_PHOTO);
      } else if (mission_items[i].camera_action == "START_PHOTO_INTERVAL") {
        wp.setCameraAction(MissionItem::CameraAction::START_PHOTO_INTERVAL);
      } else if (mission_items[i].camera_action == "STOP_PHOTO_INTERVAL") {
        wp.setCameraAction(MissionItem::CameraAction::STOP_PHOTO_INTERVAL);
      } else if (mission_items[i].camera_action == "START_VIDEO") {
        wp.setCameraAction(MissionItem::CameraAction::START_VIDEO);
      } else if (mission_items[i].camera_action == "STOP_VIDEO") {
        wp.setCameraAction(MissionItem::CameraAction::STOP_VIDEO);
      } else {
        wp.setCameraAction(MissionItem::CameraAction::NONE);
      }
    }
    
    auto result = cmd_req.send();
    auto response = result.value().content;
    if (response.getCode() != 200) {
      this->_logger.error("Upload failed: %s", response.getMessage().cStr());
    } else {
      this->_logger.info("Mission uploaded successfully with %zu waypoints", mission_items.size());
    }
    
  } else if (mission_cmd == "start") {
    auto cmd_req = this->_controller_client->new_msg();
    cmd_req.content.setStartMission();
    
    auto result = cmd_req.send();
    auto response = result.value().content;
    if (response.getCode() != 200) {
      this->_logger.error("Start failed: %s", response.getMessage().cStr());
    } else {
      this->_logger.info("Mission started");
    }
    
  } else if (mission_cmd == "pause") {
    auto cmd_req = this->_controller_client->new_msg();
    cmd_req.content.setPauseMission();
    
    auto result = cmd_req.send();
    auto response = result.value().content;
    if (response.getCode() != 200) {
      this->_logger.error("Pause failed: %s", response.getMessage().cStr());
    } else {
      this->_logger.info("Mission paused");
    }
    
  } else if (mission_cmd == "clear") {
    auto cmd_req = this->_controller_client->new_msg();
    cmd_req.content.setClearMission();
    
    auto result = cmd_req.send();
    auto response = result.value().content;
    if (response.getCode() != 200) {
      this->_logger.error("Clear failed: %s", response.getMessage().cStr());
    } else {
      this->_logger.info("Mission cleared");
    }
  } else {
    this->_logger.warn("Unknown mission command: %s", mission_cmd.c_str());
    this->_logger.info("Available commands: upload, start, pause, clear");
  }
}

std::vector<SimpleMissionItem> Commander::load_mission_from_file(const std::string& filename) {
  std::vector<SimpleMissionItem> items;
  
  try {
    YAML::Node config = YAML::LoadFile(filename);
    this->_logger.info("Parsing YAML mission file: %s", filename.c_str());
    
    if (!config["mission_items"]) {
      this->_logger.error("No 'mission_items' array found in YAML file");
      return items;
    }
    
    for (const auto& item_node : config["mission_items"]) {
      SimpleMissionItem item;
      
      item.latitude = item_node["latitude"].as<double>(0.0);
      item.longitude = item_node["longitude"].as<double>(0.0);
      item.altitude = item_node["relative_altitude"].as<float>(0.0f);
      item.speed = item_node["speed"].as<float>(5.0f);
      item.is_fly_through = item_node["is_fly_through"].as<bool>(false);
      
      item.camera_action = item_node["camera_action"].as<std::string>("NONE");
      item.loiter_time = item_node["loiter_time"].as<float>(0.0f);
      item.gimbal_pitch = item_node["gimbal_pitch"].as<float>(0.0f);
      item.gimbal_yaw = item_node["gimbal_yaw"].as<float>(0.0f);
      item.camera_photo_interval = item_node["camera_photo_interval"].as<float>(0.0f);
      
      items.push_back(item);
    }
    
    this->_logger.info("Loaded %zu waypoints from YAML file: %s", items.size(), filename.c_str());
    
  } catch (const YAML::Exception& e) {
    this->_logger.error("Failed to parse YAML file %s: %s", filename.c_str(), e.what());
    return items;
  } catch (const std::exception& e) {
    this->_logger.error("Error loading mission file %s: %s", filename.c_str(), e.what());
    return items;
  }
  
  return items;
}

int main(int argc, char **argv) {
  argparse::ArgumentParser program("mavlink-commander");
  
  program.add_argument("--registry-uri")
    .default_value("tcp://127.0.0.1:4020")
    .help("IP where the registry is running");
    
  program.add_argument("--log-level")
    .default_value(2)
    .scan<'i', int>()
    .help("The log level for the logs");
    
  program.add_argument("--debug")
    .flag()
    .help("Shortcut for --log-level 0");
    
  program.add_argument("--instance-id")
    .default_value("")
    .help("Unique identifier for this instance");
    
  program.add_argument("--ignore-heartbeat")
    .flag()
    .help("Do not verify registry heartbeat");

  argparse::ArgumentParser upload_cmd("upload");
  upload_cmd.add_description("Upload a mission from YAML file");
  upload_cmd.add_argument("file")
    .help("Mission file in YAML format");

  argparse::ArgumentParser start_cmd("start");
  start_cmd.add_description("Start the uploaded mission");

  argparse::ArgumentParser pause_cmd("pause");
  pause_cmd.add_description("Pause the running mission");

  argparse::ArgumentParser clear_cmd("clear");
  clear_cmd.add_description("Clear the current mission");

  argparse::ArgumentParser interactive_cmd("interactive");
  interactive_cmd.add_description("Start interactive mode (default)");

  program.add_subparser(upload_cmd);
  program.add_subparser(start_cmd);
  program.add_subparser(pause_cmd);
  program.add_subparser(clear_cmd);
  program.add_subparser(interactive_cmd);

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  char* minimal_argv[] = {argv[0]};
  Core::BaseArgumentParser parser(1, minimal_argv);
  std::shared_ptr<Commander> commander = std::make_shared<Commander>(parser);

  if (program.is_subcommand_used(upload_cmd)) {
    std::string filename = upload_cmd.get<std::string>("file");
    std::vector<std::string> mission_args = {"upload", filename};
    commander->handle_mission_command(mission_args);
    
  } else if (program.is_subcommand_used(start_cmd)) {
    std::vector<std::string> mission_args = {"start"};
    commander->handle_mission_command(mission_args);
    
  } else if (program.is_subcommand_used(pause_cmd)) {
    std::vector<std::string> mission_args = {"pause"};
    commander->handle_mission_command(mission_args);
    
  } else if (program.is_subcommand_used(clear_cmd)) {
    std::vector<std::string> mission_args = {"clear"};
    commander->handle_mission_command(mission_args);
    
  } else {
    std::cout << "Starting interactive mode. Type 'help' for available commands." << std::endl;
    commander->run();
  }

  return 0;
}

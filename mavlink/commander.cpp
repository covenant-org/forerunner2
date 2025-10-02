#include "argument_parser.hpp"
#include "commander.hpp"
#include <argparse/argparse.hpp>
#include <sstream>
#include <vector>
#include <array>
#include <algorithm>
#include <fstream>

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

              // Update last used values
              this->_last_global.pos.lat = lat;
              this->_last_global.pos.lon = lon;
              this->_last_global.pos.alt = alt;
              this->_last_global.yaw = yaw;

              // Use MAVSDK's native goto_location instead of manual conversion
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
      std::cout << "  mission upload <file.json>  - Upload mission from JSON file" << std::endl;
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

// Handle mission commands (extracted for reuse in CLI mode)
void Commander::handle_mission_command(const std::vector<std::string>& args) {
  if (args.size() < 1) {
    this->_logger.warn("Usage: mission <upload|start|pause|clear> [args...]");
    return;
  }
  
  std::string mission_cmd = args[0];
  
  if (mission_cmd == "upload") {
    if (args.size() < 2) {
      this->_logger.warn("Usage: mission upload <waypoint_file.json>");
      return;
    }
    
    // Load mission from file
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
      
      // Convert camera action string to enum
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

// Helper function for loading missions from file
std::vector<SimpleMissionItem> Commander::load_mission_from_file(const std::string& filename) {
  std::vector<SimpleMissionItem> items;
  std::ifstream file(filename);
  
  if (!file.is_open()) {
    this->_logger.error("Cannot open mission file: %s", filename.c_str());
    return items;
  }
  
  // Read entire file content
  std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();
  
  // Check if it's JSON format (simple heuristic)
  if (content.find("{") != std::string::npos && content.find("mission_items") != std::string::npos) {
    // Parse JSON format
    this->_logger.info("Parsing JSON mission file: %s", filename.c_str());
    
    // Simple JSON parser for our specific format
    size_t mission_items_pos = content.find("\"mission_items\"");
    if (mission_items_pos == std::string::npos) {
      this->_logger.error("No 'mission_items' array found in JSON");
      return items;
    }
    
    size_t array_start = content.find("[", mission_items_pos);
    size_t array_end = content.find("]", array_start);
    
    if (array_start == std::string::npos || array_end == std::string::npos) {
      this->_logger.error("Invalid JSON array format");
      return items;
    }
    
    std::string array_content = content.substr(array_start + 1, array_end - array_start - 1);
    
    // Parse each item in the array
    size_t pos = 0;
    while (pos < array_content.length()) {
      size_t item_start = array_content.find("{", pos);
      if (item_start == std::string::npos) break;
      
      size_t item_end = array_content.find("}", item_start);
      if (item_end == std::string::npos) break;
      
      std::string item_content = array_content.substr(item_start + 1, item_end - item_start - 1);
      
      SimpleMissionItem item;
      
      // Parse each field
      auto parse_double = [&](const std::string& field) -> double {
        size_t field_pos = item_content.find("\"" + field + "\"");
        if (field_pos != std::string::npos) {
          size_t colon_pos = item_content.find(":", field_pos);
          size_t comma_pos = item_content.find(",", colon_pos);
          if (comma_pos == std::string::npos) comma_pos = item_content.length();
          
          std::string value_str = item_content.substr(colon_pos + 1, comma_pos - colon_pos - 1);
          // Remove whitespace
          value_str.erase(std::remove_if(value_str.begin(), value_str.end(), ::isspace), value_str.end());
          return std::stod(value_str);
        }
        return 0.0;
      };
      
      auto parse_bool = [&](const std::string& field) -> bool {
        size_t field_pos = item_content.find("\"" + field + "\"");
        if (field_pos != std::string::npos) {
          size_t colon_pos = item_content.find(":", field_pos);
          std::string value_str = item_content.substr(colon_pos + 1, 10);
          return value_str.find("true") != std::string::npos;
        }
        return false;
      };
      
      auto parse_string = [&](const std::string& field) -> std::string {
        size_t field_pos = item_content.find("\"" + field + "\"");
        if (field_pos != std::string::npos) {
          size_t colon_pos = item_content.find(":", field_pos);
          size_t quote1 = item_content.find("\"", colon_pos);
          size_t quote2 = item_content.find("\"", quote1 + 1);
          if (quote1 != std::string::npos && quote2 != std::string::npos) {
            return item_content.substr(quote1 + 1, quote2 - quote1 - 1);
          }
        }
        return "NONE";
      };
      
      item.latitude = parse_double("latitude");
      item.longitude = parse_double("longitude");
      item.altitude = parse_double("relative_altitude");
      item.speed = parse_double("speed");
      item.is_fly_through = parse_bool("is_fly_through");
      item.camera_action = parse_string("camera_action");
      item.loiter_time = parse_double("loiter_time");
      
      items.push_back(item);
      pos = item_end + 1;
    }
    
  } else {
    // Parse old text format
    this->_logger.info("Parsing text mission file: %s", filename.c_str());
    std::istringstream content_stream(content);
    std::string line;
    
    while (std::getline(content_stream, line)) {
      // Skip empty lines and comments
      if (line.empty() || line[0] == '#') continue;
      
      std::istringstream iss(line);
      SimpleMissionItem item;
      
      // Format: lat lon alt speed is_fly_through
      if (iss >> item.latitude >> item.longitude >> item.altitude >> item.speed >> item.is_fly_through) {
        items.push_back(item);
      }
    }
  }
  
  this->_logger.info("Loaded %zu waypoints from %s", items.size(), filename.c_str());
  return items;
}

int main(int argc, char **argv) {
  // Create argument parser with subcommands
  argparse::ArgumentParser program("mavlink-commander");
  
  // Add global options
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

  // Add subcommands for mission operations
  argparse::ArgumentParser upload_cmd("upload");
  upload_cmd.add_description("Upload a mission from JSON file");
  upload_cmd.add_argument("file")
    .help("Mission file in JSON format");

  argparse::ArgumentParser start_cmd("start");
  start_cmd.add_description("Start the uploaded mission");

  argparse::ArgumentParser pause_cmd("pause");
  pause_cmd.add_description("Pause the running mission");

  argparse::ArgumentParser clear_cmd("clear");
  clear_cmd.add_description("Clear the current mission");

  argparse::ArgumentParser interactive_cmd("interactive");
  interactive_cmd.add_description("Start interactive mode (default)");

  // Add subcommands to main parser
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

  // Create Commander instance with minimal parser to avoid double parsing
  // We'll create a minimal argv array with just the program name
  char* minimal_argv[] = {argv[0]};
  Core::BaseArgumentParser parser(1, minimal_argv);
  std::shared_ptr<Commander> commander = std::make_shared<Commander>(parser);

  // Handle subcommands
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
    // Default to interactive mode
    std::cout << "Starting interactive mode. Type 'help' for available commands." << std::endl;
    commander->run();
  }

  return 0;
}

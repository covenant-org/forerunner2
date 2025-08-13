#include "argument_parser.hpp"
#include "commander.hpp"

Commander::Commander(Core::ArgumentParser parser) : Core::Vertex(parser) {
  this->_mission_client =
      this->create_action_client<MissionCommand, GenericResponse>("mission_command");
}

void Commander::run() {
  while (true) {
    std::string command;
    std::cout << "> " << std::flush;
    std::cin >> command;

    this->_logger.debug("Received command: %s", command.c_str());

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
    }
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser parser(argc, argv);
  std::shared_ptr<Commander> commander = std::make_shared<Commander>(parser);
  commander->run();
  return 0;
}

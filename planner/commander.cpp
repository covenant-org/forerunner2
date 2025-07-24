#include "commander.hpp"

void Commander::run() {
  while (true) {
    std::string command;
    std::cout << "> ";
    std::cin >> command;

    if (command == "takeoff") {
      auto msg = _takeoff_publisher->new_msg();
      msg.content.setAltitude(3.0);
      msg.publish();
    } else if (command == "land") {
      auto action = this->_controller_client->new_msg();
      action.content.setLand();
      auto res = action.send();
      if (res.has_value()) {
        printf("Landing status: %s\n", res.value().content.getMessage().cStr());
      }
      auto msg = _land_publisher->new_msg();
      msg.publish();
    } else if (command == "start_offboard") {
      auto msg = _start_offboard_publisher->new_msg();
      msg.content.getPos().setX(0.0f);
      msg.content.getPos().setY(0.0f);
      msg.content.getPos().setZ(0.0f);
      msg.publish();
    } else if (command == "stop_offboard") {
      auto msg = _stop_offboard_publisher->new_msg();
      msg.publish();
    }
  }
}

int main(int argc, char **argv) {
  std::shared_ptr<Commander> commander =
      std::make_shared<Commander>(argc, argv);
  commander->run();
  return 0;
}

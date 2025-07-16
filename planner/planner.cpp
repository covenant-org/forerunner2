#include "planner.hpp"
#include <iostream>
#include <memory>

Planner::Planner(int argc, char **argv) : Core::Vertex(argc, argv) {
  this->_takeoff_publisher = this->create_publisher<Takeoff>("takeoff");
  this->_land_publisher = this->create_publisher<Land>("land");
}

void Planner::run() {
  while (true) {
    std::string command;
    std::cout << "> ";
    std::cin >> command;

    if (command == "takeoff") {
      auto msg = _takeoff_publisher->new_msg();
      msg.content.setAltitude(3.0);
      msg.publish();
    } else if (command == "land") {
      auto msg = _land_publisher->new_msg();
      msg.publish();
    }
  }
}

int main(int argc, char **argv) {
  std::shared_ptr<Planner> planner = std::make_shared<Planner>(argc, argv);
  planner->run();
  return 0;
}

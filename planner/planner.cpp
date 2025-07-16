#include "message.hpp"
#include "planner.hpp"
#include <iostream>
#include <memory>

Planner::Planner(int argc, char **argv) : Core::Vertex(argc, argv) {
  this->_takeoff_publisher = this->create_publisher<Takeoff>("takeoff");
  this->_land_publisher = this->create_publisher<Land>("land");
  this->_start_offboard_publisher =
      this->create_publisher<StartOffboard>("start_offboard");
  this->_stop_offboard_publisher =
      this->create_publisher<StopOffboard>("stop_offboard");

  this->_home_position_sub = this->create_subscriber<HomePosition>(
      "home_position",
      std::bind(&Planner::home_position_cb, this, std::placeholders::_1));
}

void Planner::home_position_cb(const Core::IncomingMessage<HomePosition> &msg) {
  this->_home_position(0) = msg.content.getPos().getX();
  this->_home_position(1) = msg.content.getPos().getY();
  this->_home_position(2) = msg.content.getPos().getZ();
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
  std::shared_ptr<Planner> planner = std::make_shared<Planner>(argc, argv);
  planner->run();
  return 0;
}

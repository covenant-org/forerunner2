#include "argument_parser.hpp"
#include "crazybridge.hpp"
#include "crazyflie_cpp/Crazyflie.h"
#include <cstdio>
#include <functional>

Logger logger;

CrazyBridge::CrazyBridge(Core::ArgumentParser args)
    : Core::Vertex(args),
      crf("radio://0/100/2M/E7E7E7E7E7", logger,
          std::bind(&CrazyBridge::cb, this, std::placeholders::_1)) {}

void CrazyBridge::run() {
  printf("%s\n", this->crf.getFirmwareVersion().c_str());
  this->crf.takeoff(1, 1);
  this->crf.land(0, 1);
}

void CrazyBridge::cb(const char*) {}

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  CrazyBridge node(args);
  node.run();
  return 0;
}

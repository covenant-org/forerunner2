#include "argument_parser.hpp"
#include "crazyflie_cpp/Crazyflie.h"
#include "publisher.hpp"
#include "vertex.hpp"

#ifndef GZ_HPP
#define GZ_HPP

class CrazyBridge : public Core::Vertex {
 private:
  Crazyflie crf;
  void cb(const char*);

 public:
  CrazyBridge(Core::ArgumentParser);
  void run();
};

#endif

#include "vertex.hpp"
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <zmq.hpp>

#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

namespace Core {
template <typename T>
class Publisher {
 private:
  std::string _topic;
  uint32_t _port;

 public:
  Publisher(const std::string& topic) : _topic(std::move(topic)), _port(0) {
    this->notify_registry("")
  }
  bool notify_registry(const std::string& uri) {}

  bool setup(const std::string& uri) { zmq::context_t _context; }
  uint32_t publish(T&);
};
};  // namespace Core

#endif  // PUBLISHER_HPP
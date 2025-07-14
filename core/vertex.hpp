#include "publisher.hpp"
#include <map>
#include <memory>
#include <string>
#ifndef VERTEX_HPP
#define VERTEX_HPP

namespace Core {
class Vertex {
 private:
  static std::map<std::string, std::string> _args;

  void parse_args(int, char **) {}

 protected:
  std::string _registry;

 public:
  Vertex(int argc, char **argv) : _registry("tcp://0.0.0.0:5550") {
    this->parse_args(argc, argv);
  }

  template <typename T>
  std::shared_ptr<Publisher<T>> create_publisher(const std::string &topic, uint32_t port) {
    auto publisher = std::make_shared<Publisher<T>>(topic, port);
    publisher->setup(this->_registry);
    return publisher;
  }

  virtual void run() = 0;
};
};  // namespace Core

#endif  // VERTEX_HPP

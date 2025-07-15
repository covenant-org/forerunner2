#include "publisher.hpp"
#include <map>
#include <memory>
#include <string>
#ifndef VERTEX_HPP
#define VERTEX_HPP

// TODO: We probably should make this a global define and allow it to change
// when creating publishers
#define DEFAULT_REGISTRY_URI "tcp://127.0.0.1:4020"

namespace Core {
class Vertex {
 private:
  static std::map<std::string, std::string> _args;

  void parse_args(int, char **) {}

 protected:
  std::string _registry;

 public:
  Vertex(int argc, char **argv) : _registry(DEFAULT_REGISTRY_URI) {
    this->parse_args(argc, argv);
  }

  template <typename T>
  std::shared_ptr<Publisher<T>> create_publisher(const std::string &topic) {
    auto publisher = std::make_shared<Publisher<T>>(topic);
    publisher->setup(this->_registry);
    return publisher;
  }

  virtual void run() = 0;
};
};  // namespace Core
#endif  // VERTEX_HPP

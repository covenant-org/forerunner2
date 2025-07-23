#include "action.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include <map>
#include <memory>
#include <string>
#include <variant>
#ifndef VERTEX_HPP
#define VERTEX_HPP

// TODO: We probably should make this a global define and allow it to change
// when creating publishers
#define DEFAULT_REGISTRY_URI "tcp://127.0.0.1:4020"

namespace Core {
class Vertex {
 private:
  static std::map<std::string, std::string> _args;

  void parse_args(int, char **argv) {
    auto classname = std::string(argv[0]);
    auto last_seg = classname.find_last_of('/');
    if (last_seg != std::variant_npos) {
      classname = std::string(classname.data() + last_seg + 1);
    }
    this->_logger.set_classname(classname);
  }

 protected:
  std::string _registry;
  Logger _logger;

 public:
  Vertex(int argc, char **argv)
      : _registry(DEFAULT_REGISTRY_URI),
        _logger(LogLevel::INFO, "app.log", typeid(this).name()) {
    this->parse_args(argc, argv);
  }

  template <typename T>
  std::shared_ptr<Publisher<T>> create_publisher(const std::string &topic) {
    auto publisher = std::make_shared<Publisher<T>>(topic);
    publisher->setup(this->_registry);
    return publisher;
  }

  template <typename T>
  std::shared_ptr<Subscriber<T>> create_subscriber(
      const std::string &topic,
      std::function<void(IncomingMessage<T>)> callback) {
    auto subscriber = std::make_shared<Subscriber<T>>(topic, callback);
    subscriber->setup(this->_registry);
    return subscriber;
  }

  template <typename T, typename K>
  std::shared_ptr<ActionServer<T, K>> create_action_server(
      const std::string &topic,
      std::function<void(IncomingMessage<T>, typename K::Builder &)> callback) {
    auto server = std::make_shared<ActionServer<T, K>>(topic, callback);
    server->setup(this->_registry);
    return server;
  }

  template <typename T, typename K>
  std::shared_ptr<ActionClient<T, K>> create_action_client(
      const std::string &topic) {
    auto client = std::make_shared<ActionClient<T, K>>(topic);
    client->setup(this->_registry);
    return client;
  }

  virtual void run() = 0;
};
};  // namespace Core
#endif  // VERTEX_HPP

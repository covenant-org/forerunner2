#include "action.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include <argparse/argparse.hpp>
#include <map>
#include <memory>
#include <string>
#include <variant>
#ifndef VERTEX_HPP
#define VERTEX_HPP

#define DEFAULT_REGISTRY_URI "tcp://127.0.0.1:4020"

namespace Core {
class Vertex {
 private:
  static std::map<std::string, std::string> _args;
  bool _parsed = false;

  void parse_args(int argc, char **argv) {
    if (_parsed) return;

    setup_arguments();
    _program.parse_args(argc, argv);
    _parsed = true;

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
  argparse::ArgumentParser _program;

  virtual void setup_arguments() {}

 public:
  Vertex(int argc, char **argv)
      : _registry(DEFAULT_REGISTRY_URI),
        _logger(LogLevel::INFO, "app.log", typeid(this).name()) {
    _program.add_argument("--registry-uri")
        .default_value(DEFAULT_REGISTRY_URI)
        .help("ip where the registry is running")
        .nargs(1);
    _program.add_argument("--log_level")
        .default_value(LogLevel::INFO)
        .help("The log lever for the logs")
        .nargs(1);
    _program.add_argument("--debug").default_value(false).help(
        "Shortcut for --log_level 0");
    this->initialize(argc, argv);
  }

  void initialize(int argc, char **argv) {
    parse_args(argc, argv);
    _registry = _program.get("--registry-uri");
    if (_program.is_used("debug")) {
      this->_logger.set_level(LogLevel::DEBUG);
    }
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

#include "action.hpp"
#include "argument_parser.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include <argparse/argparse.hpp>
#include <memory>
#include <string>
#ifndef VERTEX_HPP
#define VERTEX_HPP

#define DEFAULT_REGISTRY_URI "tcp://127.0.0.1:4020"

namespace Core {
class BaseArgumentParser : public ArgumentParser {
 public:
  BaseArgumentParser(int argc, char **argv) : ArgumentParser(argc, argv) {
    this->add_argument("--registry-uri")
        .default_value(DEFAULT_REGISTRY_URI)
        .help("ip where the registry is running")
        .nargs(1);
    this->add_argument("--log-level")
        .default_value(LogLevel::INFO)
        .action(
            [](const std::string &value) { return Logger::parse_level(value); })
        .help("The log lever for the logs")
        .nargs(1);
    this->add_argument("--debug")
        .default_value(false)
        .implicit_value(true)
        .help("Shortcut for --log_level 0");
  }
};

class Vertex {
 protected:
  ArgumentParser _args;
  std::string _registry;
  Logger _logger;

 public:
  Vertex(ArgumentParser args)
      : _registry(DEFAULT_REGISTRY_URI),
        _logger(LogLevel::INFO, "app.log", typeid(this).name()),
        _args(args) {
    _args.parse();
    _logger.set_classname(this->_args._program_name);
    _registry = _args.get_argument("--registry-uri");
    auto level = _args.get_argument<LogLevel>("--log-level");
    if (_args._program->is_used("debug")) level = LogLevel::DEBUG;
    _logger.set_level(level);
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

  template <typename T = std::string>
  T get_argument(std::string_view arg_name) const {
    return this->_args.get_argument<T>(arg_name);
  }

  virtual void run() {}
};
};      // namespace Core
#endif  // VERTEX_HPP

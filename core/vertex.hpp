#include "action.hpp"
#include "argument_parser.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "utils.hpp"
#include <argparse/argparse.hpp>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>
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
        .help("Shortcut for --log-level 0");
    this->add_argument("--instance-id")
        .default_value("")
        .help(
            "Unique identifier for this instance (useful for distinguishing "
            "duplicate processes)")
        .nargs(1);
    this->add_argument("--ignore-heartbeat")
        .help("Do not verify registry heartbeat")
        .default_value(false)
        .implicit_value(true)
        .flag();
  }
};

class BaseVertex {
 protected:
  ArgumentParser _args;
  Logger _logger;

 public:
  BaseVertex(ArgumentParser args)
      : _args(args), _logger(LogLevel::INFO, "app.log", typeid(this).name()) {
    _args.parse();

    // Set logger classname with instance ID if provided
    std::string class_name = this->_args._program_name;
    std::string instance_id = _args.get_argument("--instance-id");
    if (!instance_id.empty()) {
      class_name += instance_id;
    }
    _logger.set_classname(class_name);

    auto level = _args.get_argument<LogLevel>("--log-level");
    if (_args._program->is_used("debug")) level = LogLevel::DEBUG;
    _logger.set_level(level);
  }

  template <typename T = std::string>
  T get_argument(std::string_view arg_name) const {
    return this->_args.get_argument<T>(arg_name);
  }

  virtual void run() {
    while (true) {
      sleep(1);
    }
  }
};

class Vertex : public BaseVertex {
 protected:
  std::string _registry;
  std::map<std::string, std::shared_ptr<ISubscriber>> _subscribed_topics;
  std::vector<std::shared_ptr<ISender>> _publishers;
  std::shared_ptr<Subscriber<RegistryNotification>> _sub_registry;
  std::shared_ptr<Subscriber<RegistryNotification>> _sub_heartbeat;
  std::thread _heartbeat_thread;
  std::chrono::time_point<std::chrono::system_clock> _last_registry_heartbeat;

  void run_heartbeat() {
    RateKeeper keeper(1);
    while (true) {
      auto duration = std::chrono::high_resolution_clock::now() -
                      this->_last_registry_heartbeat;
      auto seconds =
          std::chrono::duration_cast<std::chrono::seconds>(duration).count();
      if (seconds > 0) {
        this->_logger.error("Registry heartbeat timeout");
        for (const std::shared_ptr<ISender> &publisher : this->_publishers) {
          publisher->reset_connection(this->_registry);
        }
        this->_logger.info("Registry back online");
        _last_registry_heartbeat = std::chrono::high_resolution_clock::now();
      }
      keeper.keep();
    }
  }

 public:
  Vertex(ArgumentParser args)
      : BaseVertex(args), _registry(DEFAULT_REGISTRY_URI) {
    _last_registry_heartbeat = std::chrono::high_resolution_clock::now();
    _registry = _args.get_argument("--registry-uri");
    _sub_registry = std::make_shared<Subscriber<RegistryNotification>>(
        "registry/notifications",
        std::bind(&Vertex::notification_cb, this, std::placeholders::_1));
    this->_sub_registry->setup(_registry);
  }

  void notification_cb(const IncomingMessage<RegistryNotification> &msg) {
    if (msg.content.getType() == RegistryNotificationType::NODE_CHANGE) {
      auto node = msg.content.getNodeAdded().getPath();
      auto path = std::string(node.cStr(), node.size());
      try {
        auto sub = this->_subscribed_topics[path];
        if (sub.get() == nullptr) return;
        this->_logger.debug("Received node added event for %s", path.c_str());
        sub->reset_connection();
      } catch (const std::out_of_range &) {
        return;
      }
    }
    if (msg.content.getType() == RegistryNotificationType::HEARTBEAT) {
      this->_last_registry_heartbeat =
          std::chrono::high_resolution_clock::now();
    }
  }

  template <typename T>
  std::shared_ptr<Publisher<T>> create_publisher(const std::string &topic) {
    _logger.debug("Creating '%s' publisher", topic.c_str());
    auto publisher = std::make_shared<Publisher<T>>(topic);
    publisher->setup(this->_registry);
    if (this->_publishers.size() == 0) {
      if (!this->_args.get_argument<bool>("--ignore-heartbeat")) {
        this->_heartbeat_thread =
            std::thread(std::bind(&Vertex::run_heartbeat, this));
      }
    }
    this->_publishers.push_back(publisher);
    _logger.debug("Setup ready for '%s' publisher", topic.c_str());
    return publisher;
  }

  template <typename T>
  std::shared_ptr<Subscriber<T>> create_subscriber(
      const std::string &topic,
      std::function<void(IncomingMessage<T>)> callback) {
    auto subscriber = std::make_shared<Subscriber<T>>(
        topic, callback, this->_logger.get_classname());
    subscriber->setup(this->_registry);
    subscriber->set_loglevel(this->_logger.get_level());
    this->_subscribed_topics[topic] = subscriber;
    return subscriber;
  }

  template <typename T, typename K>
  std::shared_ptr<ActionServer<T, K>> create_action_server(
      const std::string &topic,
      std::function<void(IncomingMessage<T>, typename K::Builder &)> callback) {
    auto server = std::make_shared<ActionServer<T, K>>(topic, callback);
    server->setup(this->_registry);
    this->_publishers.push_back(server);
    return server;
  }

  template <typename T, typename K>
  std::shared_ptr<ActionClient<T, K>> create_action_client(
      const std::string &topic) {
    auto client = std::make_shared<ActionClient<T, K>>(topic);
    client->setup(this->_registry);
    this->_subscribed_topics[topic] = client;
    return client;
  }
};
};  // namespace Core
#endif  // VERTEX_HPP

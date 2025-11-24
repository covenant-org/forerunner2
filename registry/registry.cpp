#include "argument_parser.hpp"
#include "capnp_schemas/registry.capnp.h"
#include "registry.hpp"
#include "utils.hpp"
#include <argparse/argparse.hpp>
#include <arpa/inet.h>
#include <capnp/common.h>
#include <capnp/generated-header-support.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kj/common.h>
#include <kj/io.h>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <mutex>
#include <zmq.h>
#include <zmq.hpp>

namespace Core {

// Helper to color topic name using TOPIC_COLOR
inline std::string color_topic(const std::string &topic) {
  return std::string(TOPIC_COLOR) + topic + "\033[0m";
}
Registry::Registry(ArgumentParser args)
    : BaseVertex(args),
      _config({.port = (int16_t)args.get_argument<int>("--port"),
               .threads = (uint8_t)args.get_argument<int>("--threads")}),
      _ctx(_config.threads),
      _router(_ctx, ZMQ_ROUTER),
      _last_free_port(_config.port),
      _pub_notifications("registry/notifications") {}

void Registry::check_heartbeat() {
  RateKeeper keeper(1);
  while (true) {
    auto duration = std::chrono::high_resolution_clock::now() -
                    this->_last_external_registry_heartbeat;
    auto seconds =
        std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    if (seconds > 0) {
      this->_sub_registry->reset_connection();
      _last_external_registry_heartbeat =
          std::chrono::high_resolution_clock::now();
    }
    keeper.keep();
  }
}

void Registry::topics_action_cb(
    const IncomingMessage<TopicsListRequest>& request,
    TopicsListResponse::Builder& response) {
  bool include_internal = request.content.getIncludeInternal();
  std::vector<std::pair<std::string, Endpoint>> topics;
  {
    std::lock_guard<std::mutex> lock(_topics_mutex);
    topics.reserve(_topic_to_endpoint.size());
    for (const auto& entry : _topic_to_endpoint) {
      if (!include_internal && entry.first.rfind("registry/", 0) == 0)
        continue;
      topics.emplace_back(entry.first, entry.second);
    }
  }

  auto list = response.initTopics(topics.size());
  for (size_t i = 0; i < topics.size(); ++i) {
    const auto& [name, endpoint] = topics[i];
    auto item = list[i];
    item.setName(name);
    item.setHost(endpoint.host);
    item.setPort(endpoint.port);
  }
}

zmq::message_t Registry::message_from_builder(
    ::capnp::MallocMessageBuilder &message) {
  std::string ptr(
      ::capnp::computeSerializedSizeInWords(message) * sizeof(capnp::word),
      '\0');
  auto array = ::kj::arrayPtr((kj::byte *)ptr.data(), ptr.size());
  ::kj::ArrayOutputStream output(array);
  ::capnp::writePackedMessage(output, message);
  auto packed_size = output.getArray().size();

  return zmq::message_t(ptr.data(), packed_size);
}

void Registry::notify_node_change(std::string path, const Endpoint &endpoint) {
  auto msg = this->_pub_notifications.new_msg();
  msg.content.setType(RegistryNotificationType::NODE_CHANGE);
  auto node = msg.content.initNodeAdded();
  node.setPath(path);
  node.setPort(endpoint.port);
  msg.publish();
}

void Registry::notification_cb(
    const IncomingMessage<RegistryNotification> &msg) {
  if (msg.content.getType() == RegistryNotificationType::NODE_ADDED) {
    auto node = msg.content.getNodeAdded().getPath();
    auto path = std::string(node.cStr(), node.size());
    this->_logger.debug("Received node added event for %s", path.c_str());
    auto response = this->check_with_other_registries(path);
    if (!response.has_value()) return;
    auto value = response.value();
    this->_logger.debug(
        "Another query returned following values: %s at port %d",
        value.first.c_str(), value.second);
    std::string address = std::get<0>(value);
    uint port = std::get<1>(value);
    Endpoint endpoint{
        .host = address,
        .port = port,
    };
    this->notify_waiters(path, endpoint);
    this->notify_node_change(path, endpoint);
    return;
  }
  if (msg.content.getType() == RegistryNotificationType::NODE_CHANGE) {
    auto node = msg.content.getNodeAdded().getPath();
    auto path = std::string(node.cStr(), node.size());
    try {
      this->_logger.debug("Received node change event for %s", path.c_str());
      this->notify_node_change(path, {});
    } catch (const std::out_of_range &) {
      return;
    }
  }
  if (msg.content.getType() == RegistryNotificationType::HEARTBEAT) {
    this->_last_external_registry_heartbeat =
        std::chrono::high_resolution_clock::now();
  }
}

void Registry::respond_event(RouterEvent &event, zmq::message_t msg) {
  _router.send(event.identity, zmq::send_flags::sndmore);
  _router.send(zmq::message_t(), zmq::send_flags::sndmore);
  _router.send(msg, zmq::send_flags::none);
}

void Registry::notify_waiters(std::string path, const Endpoint &endpoint) {
  try {
    for (auto id : _topic_to_waiters[path]) {
      RouterEvent wait_event{
          .identity = zmq::message_t(id.data(), id.size()),
          .data = zmq::message_t(),
      };
      ::capnp::MallocMessageBuilder message;
      RegistryResponse::Builder res = message.initRoot<RegistryResponse>();
      res.setCode(200);
      auto host = res.initHost();
      host.setAddress(endpoint.host);
      host.setPort(endpoint.port);
      respond_event(wait_event, message_from_builder(message));
    }
  } catch (const std::out_of_range &) {
  }
}

void Registry::handle_request(RouterEvent event) {
  auto array = ::kj::arrayPtr((uint8_t *)event.data.data(), event.data.size());
  auto stream = ::kj::ArrayInputStream(array);
  ::capnp::PackedMessageReader message(stream);
  RegistryRequest::Reader request = message.getRoot<RegistryRequest>();

  if (request.getType() == RequestType::ADD_NODE) {
    ::capnp::MallocMessageBuilder message;
    RegistryResponse::Builder res = message.initRoot<RegistryResponse>();
    auto free_port = get_free_port();
    if (!free_port.has_value()) {
      res.setCode(408);
      res.setErrorMessage("Could not find a free port within range");
      respond_event(event, message_from_builder(message));
      return;
    }
    std::string path = request.getPath();
    Endpoint endpoint{
        .host = "127.0.0.1",
        .port = free_port.value(),
    };
    bool inserted_new = false;
    {
      std::lock_guard<std::mutex> lock(_topics_mutex);
      auto const insert_res = _topic_to_endpoint.insert_or_assign(path, endpoint);
      inserted_new = insert_res.second;
    }
    this->_logger.info("New topic: %s at %d", color_topic(path).c_str(),
                       endpoint.port);
    res.setCode(201);
    auto host = res.initHost();
    host.setAddress(endpoint.host);
    host.setPort(endpoint.port);
    respond_event(event, message_from_builder(message));
    notify_waiters(path, endpoint);
    if (!inserted_new) {
      this->notify_node_change(path, endpoint);
    } else {
      auto msg = this->_pub_notifications.new_msg();
      msg.content.setType(RegistryNotificationType::NODE_ADDED);
      auto node = msg.content.initNodeAdded();
      node.setPath(path);
      node.setPort(endpoint.port);
      msg.publish();
    }
    return;
  }

  if (request.getType() == RequestType::QUERY_NODE) {
    auto path = request.getPath();
    this->_logger.debug("querying topic: %s",
                        color_topic(request.getPath().cStr()).c_str());
    try {
      Endpoint node;
      {
        std::lock_guard<std::mutex> lock(_topics_mutex);
        node = _topic_to_endpoint.at(path);
      }
      this->_logger.debug("querying topic [%s] found at %d",
                          color_topic(request.getPath().cStr()).c_str(),
                          node.port);

      ::capnp::MallocMessageBuilder message;
      RegistryResponse::Builder res = message.initRoot<RegistryResponse>();
      res.setCode(200);
      auto host = res.initHost();
      host.setAddress(node.host);
      host.setPort(node.port);

      if (request.hasNetworks()) {
        ::capnp::MallocMessageBuilder message;
        RegistryResponse::Builder res = message.initRoot<RegistryResponse>();
        auto local_networks = get_ipv4_networks();
        auto remote_networks = request.getNetworks();
        bool found = false;
        uint32_t ip;
        for (auto remote_network : remote_networks) {
          try {
            ip = local_networks.at(remote_network.getNetwork());
            res.setCode(200);
            found = true;
            break;
          } catch (std::out_of_range &) {
          }
        }

        if (found) {
          host.setPort(node.port);
          struct in_addr network_addr;
          network_addr.s_addr = ip;
          host.setAddress(std::string(inet_ntoa(network_addr)));
          this->_logger.debug("Should connect to: %s",
                              host.getAddress().asString().cStr());
        } else {
          res.setCode(500);
          std::string error_message = "No matching local and remote networks";
          res.initErrorMessage(error_message.size());
          res.setErrorMessage(error_message);
        }
      }
      respond_event(event, message_from_builder(message));

    } catch (const std::out_of_range &) {
      ::capnp::MallocMessageBuilder message;
      RegistryResponse::Builder res = message.initRoot<RegistryResponse>();
      auto response = this->check_with_other_registries(path);
      if (response.has_value()) {
        auto value = response.value();
        this->_logger.debug(
            "another query returned following values: %s at port %d",
            value.first.c_str(), value.second);
        res.setCode(200);
        auto host = res.initHost();
        host.setAddress(std::get<0>(value));
        host.setPort(std::get<1>(value));
        respond_event(event, message_from_builder(message));
      } else {
        this->_logger.debug("querying topic not found, pending to notify");
        if (request.hasNetworks()) {
          res.setCode(404);
          std::string error_message =
              "Topic " + std::string(path.cStr(), path.size()) + " not found";
          res.initErrorMessage(error_message.size());
          res.setErrorMessage(error_message);
          respond_event(event, message_from_builder(message));
          return;
        }
        _topic_to_waiters[path].emplace_back((char *)event.identity.data(),
                                             event.identity.size());
      }
    }
    return;
  }
}

std::optional<std::pair<std::string, uint32_t>>
Registry::check_with_other_registries(const std::string &path) {
  std::string other_registry =
      this->get_argument<std::string>("--registry-uri");
  if (other_registry.length()) {
    this->_logger.debug("Querying registry at %s for topic %s",
                        other_registry.c_str(), path.c_str());
    return query_another_registry(path, other_registry);
  }

  return std::nullopt;
}

std::optional<uint32_t> Registry::get_free_port() {
  zmq::context_t tmp_ctx(1);
  zmq::socket_t tmp(tmp_ctx, ZMQ_REQ);
  for (int i = 1; i < MAX_PORT_SEARCHES; i++) {
    char bind_dir[20];
    sprintf(bind_dir, "tcp://0.0.0.0:%d", _last_free_port + i);
    try {
      tmp.bind(bind_dir);
      tmp.unbind(bind_dir);
      _last_free_port += i;
      return _last_free_port;
    } catch (const zmq::error_t &error) {
      this->_logger.error("get_free_port failed due to: %s", error.what());
    }
  }
  return std::nullopt;
}

std::optional<RouterEvent> Registry::wait_for_message(zmq::socket_t &) {
  // Router gets the identity first
  zmq::message_t identity(5);
  auto res = _router.recv(identity);
  if (res.value_or(0) == 0) {
    this->_logger.error("Bad message while waiting for id");
    return std::nullopt;
  }

  zmq::message_t empty;
  res = _router.recv(empty);
  if (res.value_or(0) != 0) {
    this->_logger.error("Bad message while waiting for package separator");
    return std::nullopt;
  }

  zmq::message_t data(1024);
  res = _router.recv(data);
  if (res.has_value()) {
    return std::optional<RouterEvent>(
        RouterEvent{.identity = std::move(identity), .data = std::move(data)});
  }
  return std::nullopt;
}

void Registry::run() {
  char bind_dir[20];
  sprintf(bind_dir, "tcp://*:%d", _config.port);
  _router.bind(std::string(bind_dir));
  this->_logger.info("Listening to %s", bind_dir);

  auto port = this->get_free_port();
  if (!port.has_value()) {
    throw std::runtime_error(
        "Cannot find free port for registry notifications");
  }
  this->_pub_notifications._bind_to_port(port.value());
  auto notifications_topic = this->_pub_notifications.get_topic();
  {
    std::lock_guard<std::mutex> lock(_topics_mutex);
    this->_topic_to_endpoint.insert_or_assign(
        notifications_topic, Endpoint{.host = "127.0.0.1",
                                       .port = port.value()});
  }
  this->_logger.debug("Registry %s on %d", notifications_topic.c_str(),
                      port.value());

  _topics_action = std::make_shared<ActionServer<TopicsListRequest, TopicsListResponse>>(
      "registry/topics",
      [this](IncomingMessage<TopicsListRequest> request,
             TopicsListResponse::Builder& response) {
        this->topics_action_cb(request, response);
      });
  std::string registry_uri = "tcp://127.0.0.1:" + std::to_string(_config.port);
  std::thread topics_action_setup([this, registry_uri]() {
    try {
      this->_topics_action->setup(registry_uri);
    } catch (const std::exception& ex) {
      this->_logger.error("Failed to start topics action: %s", ex.what());
    }
  });
  topics_action_setup.detach();

  std::string external_uri =
      this->_args.get_argument<std::string>("--registry-uri");
  if (external_uri.size() > 0) {
    this->_sub_registry = std::make_shared<Subscriber<RegistryNotification>>(
        "registry/notifications",
        std::bind(&Registry::notification_cb, this, std::placeholders::_1));
    this->_sub_registry->setup(external_uri);
    this->_external_heartbeat_thread =
        std::thread(std::bind(&Registry::check_heartbeat, this));
  }

  this->_heartbeat_thread = std::thread(std::bind(&Registry::heartbeat, this));

  while (true) {
    auto event = wait_for_message(_router);
    if (!event.has_value()) continue;
    handle_request(std::move(event.value()));
  }
}

void Registry::heartbeat() {
  RateKeeper ratekeeper(3);
  while (true) {
    auto msg = this->_pub_notifications.new_msg();
    msg.content.setType(RegistryNotificationType::HEARTBEAT);
    msg.content.setHeartbeat();
    msg.publish();
    ratekeeper.keep();
  }
}

}  // namespace Core

int main(int argc, char **argv) {
  Core::BaseArgumentParser args(argc, argv);
  args.add_argument("--port", "-p")
      .help("Port to bind")
      .default_value(4020)
      .scan<'i', int>();
  args.add_argument("--threads", "-j")
      .help("Number of zmq threads to handle requets")
      .default_value(5)
      .scan<'i', int>();
  args.add_argument("--registry-uri", "-r")
      .help("other registry nodes host with port")
      .default_value("");
  Core::Registry registry(args);
  registry.run();
  return 0;
}

#ifndef REGISTRY_HPP
#define REGISTRY_HPP

#include "argument_parser.hpp"
#include <capnp/message.h>
#include <capnp_schemas/registry.capnp.h>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <vertex.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#define MAX_PORT_SEARCHES 100
// Topic color (ANSI):
#define TOPIC_COLOR "\033[36m"  // cyan

namespace Core {
std::string color_topic(const std::string& topic);
struct Endpoint {
  std::string host;
  uint32_t port;
};

struct RegistryConfiguration {
  int16_t port;
  uint8_t threads;
};

struct RouterEvent {
  zmq::message_t identity;
  zmq::message_t data;
};

class Registry : public BaseVertex {
 public:
  explicit Registry(ArgumentParser);

  void run();
  void heartbeat();
  void register_topic(const std::string topic, const Endpoint& host);
  void deregister_topic(const std::string topic);
  void notification_cb(const IncomingMessage<RegistryNotification>&);
  std::optional<uint32_t> get_free_port();

  std::optional<Endpoint> check_topic(const std::string topic);
  void persist_topics();

  void handle_request(RouterEvent);
  std::optional<RouterEvent> wait_for_message(zmq::socket_t& socket);

  void respond_event(RouterEvent& event, zmq::message_t data);
  zmq::message_t message_from_builder(::capnp::MallocMessageBuilder& msg);
  std::optional<std::pair<std::string, uint32_t>> check_with_other_registries(
      const std::string&);
  void notify_waiters(std::string path, const Endpoint&);
  void notify_node_change(std::string path, const Endpoint&);
  void check_heartbeat();
  void topics_action_cb(const IncomingMessage<TopicsListRequest>& request,
                        TopicsListResponse::Builder& response);

 private:
  RegistryConfiguration _config;
  zmq::context_t _ctx;
  zmq::socket_t _router;
  uint32_t _last_free_port;
  std::unordered_map<std::string, Endpoint> _topic_to_endpoint;
  std::map<std::string, std::vector<std::string>> _topic_to_waiters;
  Publisher<RegistryNotification> _pub_notifications;
  std::shared_ptr<ActionServer<TopicsListRequest, TopicsListResponse>>
      _topics_action;
  std::mutex _topics_mutex;
  std::thread _heartbeat_thread;
  std::shared_ptr<Subscriber<RegistryNotification>> _sub_registry;
  std::thread _external_heartbeat_thread;
  std::chrono::time_point<std::chrono::system_clock>
      _last_external_registry_heartbeat;
};
}  // namespace Core

#endif

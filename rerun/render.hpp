#ifndef RERUN_RENDER_REGISTRY_HPP
#define RERUN_RENDER_REGISTRY_HPP

#include <capnp/any.h>
#include <capnp/dynamic.h>
#include <capnp/schema-parser.h>
#include <rerun.hpp>
#include <rerun/archetypes/points3d.hpp>

#include "../core/message.hpp"
#include "../core/dynamic_reflection.hpp"
#include "../core/vertex.hpp"
#include "viewer.hpp" // Forward declaration para Viewer

#include <capnp_schemas/geometry_msgs.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/registry.capnp.h>
#include <capnp_schemas/visualization_msgs.capnp.h>
#include <capnp_schemas/nav_msgs.capnp.h>

#include <kj/filesystem.h>

#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <functional>
#include <mutex>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Core {
class Logger;
}

namespace RerunRenderers {

class RenderContext {
 public:
  RenderContext(rerun::RecordingStream& stream,
                std::string base_path,
                Core::Logger* logger = nullptr)
      : _stream(stream),
        _base_path(std::move(base_path)),
        _logger(logger) {}

  rerun::RecordingStream& stream() { return _stream; }
  const std::string& base_path() const { return _base_path; }
  std::string make_child_path(const std::string& suffix) const {
    if (suffix.empty()) {
      return _base_path;
    }
    if (_base_path.empty()) {
      return suffix;
    }
    return _base_path + "/" + suffix;
  }

  Core::Logger* logger() const { return _logger; }

 private:
  rerun::RecordingStream& _stream;
  std::string _base_path;
  Core::Logger* _logger;
};

using RendererFn = std::function<void(const Core::EnvelopeMetadata&,
                                      ::capnp::AnyPointer::Reader,
                                      RenderContext&)>;

class RendererRegistry {
 public:
  static bool register_renderer(const std::string& type_name, RendererFn fn) {
    auto& table = instance();
    return table.map.emplace(type_name, std::move(fn)).second;
  }
  static std::optional<RendererFn> find(const std::string& type_name) {
    auto& table = instance();
    auto it = table.map.find(type_name);
    if (it == table.map.end()) {
      return std::nullopt;
    }
    return it->second;
  }

 private:
  RendererRegistry() = default;
  static RendererRegistry& instance() {
    static RendererRegistry registry;
    return registry;
  }

  std::unordered_map<std::string, RendererFn> map;
};

namespace {
// =====================
// Generic and reflection functions
// =====================
inline void log_reflected_fields(
    const std::vector<DynamicReflection::ReflectedField>& fields,
    RenderContext& context,
    const std::string& prefix) {
  for (const auto& field : fields) {
    std::string current_path = prefix.empty() ? field.name
                                              : prefix + "/" + field.name;
    if (field.is_leaf()) {
      context.stream().log(context.make_child_path(current_path),
                           rerun::TextLog(field.value));
    } else if (!field.children.empty()) {
      log_reflected_fields(field.children, context, current_path);
    } else {
      context.stream().log(context.make_child_path(current_path),
                           rerun::TextLog("(empty)"));
    }
  }
}

inline std::string to_std_string(::capnp::Text::Reader reader) {
  if (reader.size() == 0) {
    return {};
  }
  return std::string(reader.cStr(), reader.size());
}

inline void render_generic(const Core::EnvelopeMetadata& metadata,
                    ::capnp::AnyPointer::Reader payload,
                    RenderContext& context,
                    ::capnp::StructSchema schema) {
  auto dynamic_reader = payload.getAs<::capnp::DynamicStruct>(schema);
  auto reflected = DynamicReflection::reflect(dynamic_reader);

  if (reflected.empty()) {
    context.stream().log(context.make_child_path("generic"),
                         rerun::TextLog("(empty struct)"));
    return;
  }

  log_reflected_fields(reflected, context, "generic");
}

// =====================
// Point functions
// =====================
inline void render_point(const Core::EnvelopeMetadata& metadata,
                  ::capnp::AnyPointer::Reader payload,
                  RenderContext& context) {
  auto reader = payload.getAs<::Point>();

  const float x = static_cast<float>(reader.getX());
  const float y = static_cast<float>(reader.getY());
  const float z = static_cast<float>(reader.getZ());

  std::vector<rerun::Position3D> positions = {{x, y, z}};
  context.stream().log(context.make_child_path("point"),
                       rerun::Points3D(positions).with_radii({0.05F}));
}

// =====================
// Marker functions
// =====================
inline float compute_marker_radius(const ::Marker::Reader& marker) {
  const auto scale = marker.getScale();
  const float sx = static_cast<float>(scale.getX());
  const float sy = static_cast<float>(scale.getY());
  const float sz = static_cast<float>(scale.getZ());
  const float max_axis = std::max({std::abs(sx), std::abs(sy), std::abs(sz)});
  return max_axis > 0.0F ? max_axis * 0.5F : 0.05F;
}

inline std::string marker_type_label(::MarkerType shape) {
  switch (shape) {
    case ::MarkerType::CUBE:
      return "cube";
    case ::MarkerType::SPHERE:
      return "sphere";
  }
  return "unknown";
}

inline void log_marker_details(const ::Marker::Reader& marker,
                        RenderContext& context,
                        const std::string& base_segment) {
  const auto pose = marker.getPose();
  const auto position = pose.getPosition();
  const float px = static_cast<float>(position.getX());
  const float py = static_cast<float>(position.getY());
  const float pz = static_cast<float>(position.getZ());

  const float radius = compute_marker_radius(marker);
  std::vector<rerun::Position3D> positions = {{px, py, pz}};
  context.stream().log(context.make_child_path(base_segment + "/position"),
                       rerun::Points3D(positions).with_radii({radius}));

  const auto color = marker.getColor();
  std::ostringstream info;
  info << "shape=" << marker_type_label(marker.getShape())
       << ", lifetime=" << marker.getLifetime()
       << ", color rgba=(" << color.getR() << ',' << color.getG() << ','
       << color.getB() << ',' << color.getA() << ')';

  context.stream().log(context.make_child_path(base_segment + "/info"),
                       rerun::TextLog(info.str()));
}

inline void render_marker(const Core::EnvelopeMetadata& metadata,
                   ::capnp::AnyPointer::Reader payload,
                   RenderContext& context) {
  auto marker = payload.getAs<::Marker>();

  const std::string marker_ns = to_std_string(marker.getNs());
  std::string segment = "markers/" +
                        (marker_ns.empty() ? std::string("default")
                                           : marker_ns) +
                        "_" + std::to_string(marker.getId());

  log_marker_details(marker, context, segment);
}

inline void render_marker_array(const Core::EnvelopeMetadata& metadata,
                         ::capnp::AnyPointer::Reader payload,
                         RenderContext& context) {
  auto markers = payload.getAs<::MarkerArray>().getMarkers();
  uint32_t index = 0;
  for (const auto marker : markers) {
    std::string segment = "markers/array_" + std::to_string(index);
    log_marker_details(marker, context, segment);
    ++index;
  }
  if (index == 0) {
    context.stream().log(context.make_child_path("markers"),
                         rerun::TextLog("(empty marker array)"));
  }
}

// =====================
// Odometry functions
// =====================
inline void render_odometry(const Core::EnvelopeMetadata& metadata,
                     ::capnp::AnyPointer::Reader payload,
                     RenderContext& context) {
  auto odom = payload.getAs<::Odometry>();
  const auto pose = odom.getPose().getPose();
  const auto position = pose.getPosition();
  const auto orientation = pose.getOrientation();

  // Visualiza la posición como punto (con ejes invertidos)
  std::vector<std::array<float, 3>> positions = {
    {static_cast<float>(position.getX()),
     -static_cast<float>(position.getY()),
     -static_cast<float>(position.getZ())}
  };
  context.stream().log(context.make_child_path("odometry/position"),
                       rerun::Points3D(positions).with_radii({0.1F}));

  // Visualiza la pose como transformación (con ejes invertidos)
  context.stream().log("world/drone",
    rerun::Transform3D::from_translation_rotation(
      rerun::components::Translation3D(std::array<float, 3>{
        static_cast<float>(position.getX()),
        -static_cast<float>(position.getY()),
        -static_cast<float>(position.getZ())
      }),
      rerun::Rotation3D(rerun::datatypes::Quaternion{
        static_cast<float>(orientation.getX()),
        -static_cast<float>(orientation.getY()),
        -static_cast<float>(orientation.getZ()),
        static_cast<float>(orientation.getW())
      })
    )
  );
}

// ===== Point renderers registration =====
[[maybe_unused]] const bool point_registered =
  RendererRegistry::register_renderer("geometry_msgs.Point", &render_point);
[[maybe_unused]] const bool point_short_registered =
  RendererRegistry::register_renderer("Point", &render_point);

// ===== Marker renderers registration =====
[[maybe_unused]] const bool marker_registered = RendererRegistry::register_renderer(
  "visualization_msgs.Marker", &render_marker);
[[maybe_unused]] const bool marker_short_registered =
  RendererRegistry::register_renderer("Marker", &render_marker);

// ===== MarkerArray renderers registration =====
[[maybe_unused]] const bool marker_array_registered = RendererRegistry::register_renderer(
  "visualization_msgs.MarkerArray", &render_marker_array);
[[maybe_unused]] const bool marker_array_short_registered =
  RendererRegistry::register_renderer("MarkerArray", &render_marker_array);
[[maybe_unused]] const bool odometry_registered =
  RendererRegistry::register_renderer("nav_msgs.Odometry", &render_odometry);
[[maybe_unused]] const bool odometry_short_registered =
  RendererRegistry::register_renderer("Odometry", &render_odometry);
}  // namespace

inline void dispatch_message(const Core::EnvelopeMetadata& metadata,
                      ::capnp::AnyPointer::Reader payload,
                      RenderContext& context,
                      ::capnp::StructSchema schema) {
  if (auto renderer = RendererRegistry::find(metadata.typeName)) {
    if (auto* logger = context.logger()) {
      logger->debug("Rendering type '%s'", metadata.typeName.c_str());
    }
    (*renderer)(metadata, payload, context);
    return;
  }

  if (auto* logger = context.logger()) {
    logger->debug("Rendered type '%s' using generic fallback",
                  metadata.typeName.c_str());
  }
  render_generic(metadata, payload, context, schema);
}

template <typename T>
inline void dispatch_message(const Core::IncomingMessage<T>& msg,
                      RenderContext& context) {
  const ::capnp::StructSchema schema = ::capnp::Schema::from<T>().asStruct();
  dispatch_message(msg.metadata, msg.getRawPayload(), context, schema);
}

}  // namespace RerunRenderers

namespace RegistryExample {

namespace {

class SchemaCache {
 public:
  SchemaCache()
      : filesystem_(kj::newDiskFilesystem()),
        base_directory_(&filesystem_->getCurrent()) {}

    const capnp::StructSchema& resolve(const std::string& schema_path,
                                       const std::string& type_name,
                                       Core::Logger& logger) {
      const std::string key = schema_path + "::" + type_name;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = cache_.find(key);
        if (it != cache_.end()) {
          return it->second;
        }
      }

    std::vector<std::string> candidates = {DynamicReflection::canonicalize_schema_path(schema_path)};

    for (size_t i = 0; i < candidates.size(); ++i) {
      const bool last_attempt = (i + 1 == candidates.size());
      try {
        kj::ArrayPtr<const kj::ReadableDirectory* const> import_dirs;
        auto parsed = parser_.parseFromDirectory(
            *base_directory_, kj::Path::parse(candidates[i].c_str()),
            import_dirs);

        capnp::ParsedSchema current = parsed;
        size_t start = 0;
        while (start < type_name.size()) {
          const auto dot = type_name.find('.', start);
          const auto part = type_name.substr(
              start, dot == std::string::npos ? std::string::npos : dot - start);
          if (part.empty()) {
            break;
          }
          current = current.getNested(part.c_str());
          if (dot == std::string::npos) {
            break;
          }
          start = dot + 1;
        }

        const capnp::StructSchema schema = current.asStruct();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          auto [it, inserted] = cache_.emplace(key, schema);
          (void)inserted;
          logger.debug("Loaded schema %s (%s)", candidates[i].c_str(),
                       type_name.c_str());
          return it->second;
        }
      } catch (const kj::Exception& ex) {
        const char* message = ex.getDescription().cStr();
        if (last_attempt) {
          logger.warn("Failed to load schema %s (%s): %s",
                      candidates[i].c_str(),
                      type_name.c_str(),
                      message);
        }
      }
    }

    throw std::runtime_error("Failed to resolve schema " + key);
  }

 private:
  std::mutex mutex_;
  capnp::SchemaParser parser_;
  kj::Own<kj::Filesystem> filesystem_;
  const kj::Directory* base_directory_;
  std::unordered_map<std::string, capnp::StructSchema> cache_;
};

}  // namespace

class Renderer : public Core::Vertex {
public:
  Renderer(Core::ArgumentParser args,
           std::shared_ptr<rerun::RecordingStream> shared_stream,
           Viewer* viewer)
      : Core::Vertex(args),
        stream_(shared_stream),
        viewer_(viewer),
        shutting_down_(false),
        ignored_prefixes_({"registry/"}) {
    // TODO: Arreglar problema de args
    // Solo iniciar spawn si no está el flag --no-record
    // std::cout << "Renderer started" << std::endl;
    bool no_record = false;
    try {
      no_record = args.get_argument<bool>("--no-record");
    } catch (...) {}
    if (!no_record) {
      stream_->spawn().exit_on_failure();
    }
    topics_client_ = this->create_action_client<TopicsListRequest, TopicsListResponse>(
        "registry/topics");
    refresh_topics();
    refresh_thread_ = std::thread(&Renderer::topic_refresh_loop, this);
  }

  ~Renderer() {
    shutting_down_.store(true);
    if (refresh_thread_.joinable()) {
      refresh_thread_.join();
    }
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    for (auto& [topic, subscriber] : topic_subscribers_) {
      if (subscriber) {
        subscriber->stop();
      }
    }
  }

private:
  Viewer* viewer_;
  void topic_refresh_loop() {
    while (!shutting_down_.load()) {
      try {
        refresh_topics();
      } catch (const std::exception& ex) {
        _logger.warn("Topic refresh failed: %s", ex.what());
      }
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }
  }

  void refresh_topics() {
    if (!topics_client_) {
      return;
    }

    // TODO: Add white/black list
    auto request = topics_client_->new_msg();
    request.content.setIncludeInternal(false);
    auto response = request.send();
    if (!response.has_value()) {
      _logger.debug("Topics action returned no response");
      return;
    }

    auto topics = response->content.getTopics();
    std::unordered_set<std::string> active_topics;
    active_topics.reserve(topics.size());

    for (auto topic_entry : topics) {
      const auto name_reader = topic_entry.getName();
      std::string topic_name(name_reader.cStr(), name_reader.size());
      if (should_ignore_topic(topic_name)) {
        continue;
      }
      active_topics.insert(topic_name);
      ensure_subscription(topic_name);
    }

    std::vector<std::pair<std::string, std::shared_ptr<Core::Subscriber<::capnp::AnyPointer>>>>
        removals;
    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      auto it = topic_subscribers_.begin();
      while (it != topic_subscribers_.end()) {
        if (active_topics.find(it->first) == active_topics.end()) {
          removals.emplace_back(it->first, it->second);
          it = topic_subscribers_.erase(it);
        } else {
          ++it;
        }
      }
    }

    for (auto& entry : removals) {
      if (entry.second) {
        entry.second->stop();
      }
      _logger.info("Unsubscribed from topic %s", entry.first.c_str());
    }
  }

  void ensure_subscription(const std::string& topic_name) {
    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      if (topic_subscribers_.find(topic_name) != topic_subscribers_.end()) {
        return;
      }
    }

    auto subscriber = this->create_subscriber<::capnp::AnyPointer>(
        topic_name,
        [this, topic_name](const Core::IncomingMessage<::capnp::AnyPointer>& msg) {
          handle_any_message(topic_name, msg);
        });
    subscriber->set_loglevel(_logger.get_level());

    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      topic_subscribers_.emplace(topic_name, subscriber);
    }
    _logger.info("Subscribed to topic %s", topic_name.c_str());
  }

  bool should_ignore_topic(const std::string& topic_name) const {
    for (const auto& prefix : ignored_prefixes_) {
      if (topic_name.rfind(prefix, 0) == 0) {
        return true;
      }
    }
    return false;
  }

  void handle_any_message(
      const std::string& default_topic,
      const Core::IncomingMessage<::capnp::AnyPointer>& msg) {
    const capnp::StructSchema& struct_schema = schema_cache_.resolve(
        msg.metadata.schemaPath, msg.metadata.typeName, _logger);

    std::string base_path =
        msg.metadata.topic.empty() ? default_topic : msg.metadata.topic;
    RerunRenderers::RenderContext context(*stream_, std::move(base_path),
                        &_logger);
    RerunRenderers::dispatch_message(msg.metadata, msg.getRawPayload(),
                                     context, struct_schema);
  }

  std::shared_ptr<rerun::RecordingStream> stream_;
  std::shared_ptr<Core::ActionClient<TopicsListRequest, TopicsListResponse>>
      topics_client_;
  std::unordered_map<std::string,
                     std::shared_ptr<Core::Subscriber<::capnp::AnyPointer>>>
      topic_subscribers_;
  std::mutex subscribers_mutex_;
  SchemaCache schema_cache_;
  std::thread refresh_thread_;
  std::atomic_bool shutting_down_;
  std::vector<std::string> ignored_prefixes_;
};

}  // namespace RegistryExample

#endif  // RERUN_RENDER_REGISTRY_HPP

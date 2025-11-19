// Example implementation of a renderer registry with a single specialization
// for geometry_msgs::Point. The structure is intentionally modular so that
// adding more templates only requires registering another renderer function.

#include "render_registry.hpp"

#include <capnp/dynamic.h>
#include <rerun.hpp>
#include <rerun/archetypes/points3d.hpp>

#include "../core/dynamic_reflection.hpp"
#include "../core/vertex.hpp"

#include <capnp_schemas/geometry_msgs.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/visualization_msgs.capnp.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace RerunRenderers {
namespace {

void log_reflected_fields(
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

std::string to_std_string(::capnp::Text::Reader reader) {
  if (reader.size() == 0) {
    return {};
  }
  return std::string(reader.cStr(), reader.size());
}

float compute_marker_radius(const ::Marker::Reader& marker) {
  const auto scale = marker.getScale();
  const float sx = static_cast<float>(scale.getX());
  const float sy = static_cast<float>(scale.getY());
  const float sz = static_cast<float>(scale.getZ());
  const float max_axis = std::max({std::abs(sx), std::abs(sy), std::abs(sz)});
  return max_axis > 0.0F ? max_axis * 0.5F : 0.05F;
}

std::string marker_type_label(::MarkerType shape) {
  switch (shape) {
    case ::MarkerType::CUBE:
      return "cube";
    case ::MarkerType::SPHERE:
      return "sphere";
  }
  return "unknown";
}

void log_marker_details(const ::Marker::Reader& marker,
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

void render_generic(const Core::EnvelopeMetadata& metadata,
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

}  // namespace

RendererRegistry& RendererRegistry::instance() {
  static RendererRegistry registry;
  return registry;
}

bool RendererRegistry::register_renderer(const std::string& type_name,
                                         RendererFn fn) {
  auto& table = instance();
  return table.map.emplace(type_name, std::move(fn)).second;
}

std::optional<RendererFn> RendererRegistry::find(const std::string& type_name) {
  auto& table = instance();
  auto it = table.map.find(type_name);
  if (it == table.map.end()) {
    return std::nullopt;
  }
  return it->second;
}

void dispatch_message(const Core::EnvelopeMetadata& metadata,
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

// Specialized renderer for geometry_msgs::Point. Demonstrates how to plug an
// actual visualization into the registry.
void render_point(const Core::EnvelopeMetadata& metadata,
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

void render_marker(const Core::EnvelopeMetadata& metadata,
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

void render_marker_array(const Core::EnvelopeMetadata& metadata,
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

namespace {
[[maybe_unused]] const bool point_registered =
  RendererRegistry::register_renderer("geometry_msgs.Point", &render_point);
[[maybe_unused]] const bool point_short_registered =
  RendererRegistry::register_renderer("Point", &render_point);

[[maybe_unused]] const bool marker_registered = RendererRegistry::register_renderer(
  "visualization_msgs.Marker", &render_marker);
[[maybe_unused]] const bool marker_short_registered =
  RendererRegistry::register_renderer("Marker", &render_marker);

[[maybe_unused]] const bool marker_array_registered = RendererRegistry::register_renderer(
  "visualization_msgs.MarkerArray", &render_marker_array);
[[maybe_unused]] const bool marker_array_short_registered =
  RendererRegistry::register_renderer("MarkerArray", &render_marker_array);
}  // namespace

}  // namespace RerunRenderers

namespace RegistryExample {

class PointSubscriber : public Core::Vertex {
 public:
  PointSubscriber(Core::ArgumentParser args)
      : Core::Vertex(args),
        point_topic_("Point"),
        telemetry_topic_("telemetry"),
        marker_topic_("visualization/marker"),
        marker_array_topic_("visualization/markers") {
    stream_ = std::make_shared<rerun::RecordingStream>("PointSubscriber");
    stream_->spawn().exit_on_failure();

    point_subscriber_ = this->create_subscriber<::Point>(
        point_topic_,
        std::bind(&PointSubscriber::on_point_message, this,
                  std::placeholders::_1));

    telemetry_subscriber_ = this->create_subscriber<::Telemetry>(
        telemetry_topic_,
        std::bind(&PointSubscriber::on_telemetry_message, this,
                  std::placeholders::_1));

    marker_subscriber_ = this->create_subscriber<::Marker>(
        marker_topic_,
        std::bind(&PointSubscriber::on_marker_message, this,
                  std::placeholders::_1));

    marker_array_subscriber_ = this->create_subscriber<::MarkerArray>(
        marker_array_topic_,
        std::bind(&PointSubscriber::on_marker_array_message, this,
                  std::placeholders::_1));
  }

 private:
  void on_point_message(const Core::IncomingMessage<::Point>& msg) {
    std::string base_path =
        msg.metadata.topic.empty() ? point_topic_ : msg.metadata.topic;
    RerunRenderers::RenderContext context(
        *stream_, std::move(base_path), &_logger);
    RerunRenderers::dispatch_message(msg, context);
  }

  void on_telemetry_message(const Core::IncomingMessage<::Telemetry>& msg) {
    std::string base_path =
        msg.metadata.topic.empty() ? telemetry_topic_ : msg.metadata.topic;
    RerunRenderers::RenderContext context(
        *stream_, std::move(base_path), &_logger);
    RerunRenderers::dispatch_message(msg, context);
  }

  void on_marker_message(const Core::IncomingMessage<::Marker>& msg) {
    std::string base_path =
        msg.metadata.topic.empty() ? marker_topic_ : msg.metadata.topic;
    RerunRenderers::RenderContext context(
        *stream_, std::move(base_path), &_logger);
    RerunRenderers::dispatch_message(msg, context);
  }

  void on_marker_array_message(
      const Core::IncomingMessage<::MarkerArray>& msg) {
    std::string base_path =
        msg.metadata.topic.empty() ? marker_array_topic_ : msg.metadata.topic;
    RerunRenderers::RenderContext context(
        *stream_, std::move(base_path), &_logger);
    RerunRenderers::dispatch_message(msg, context);
  }

  const std::string point_topic_;
  const std::string telemetry_topic_;
  const std::string marker_topic_;
  const std::string marker_array_topic_;
  std::shared_ptr<rerun::RecordingStream> stream_;
  std::shared_ptr<Core::Subscriber<::Point>> point_subscriber_;
  std::shared_ptr<Core::Subscriber<::Telemetry>> telemetry_subscriber_;
  std::shared_ptr<Core::Subscriber<::Marker>> marker_subscriber_;
  std::shared_ptr<Core::Subscriber<::MarkerArray>> marker_array_subscriber_;
};

}  // namespace RegistryExample

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  RegistryExample::PointSubscriber app(args);
  app.run();
  return 0;
}

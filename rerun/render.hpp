#define ANY_READER ::capnp::AnyPointer::Reader
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
// Utilidad para obtener el reader correcto desde AnyPointer o Reader directo
template<typename T, typename ReaderT>
typename T::Reader resolve_reader(ReaderT reader_or_any) {
  if constexpr (std::is_same_v<ReaderT, ANY_READER>) {
    if (reader_or_any.isStruct()) {
      return reader_or_any.template getAs<T>();
    }
    // Si no es struct, retorna default constructed (puedes ajustar si quieres lanzar excepción)
    return typename T::Reader();
  } else {
    return reader_or_any;
  }
}

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
  template<typename RenderFunc>
  static RendererFn make_renderer(RenderFunc func) {
    return [func](const Core::EnvelopeMetadata& metadata, ::capnp::AnyPointer::Reader payload, RenderContext& context) {
      func(metadata, payload, context);
    };
  }
  // Sobrecarga para aceptar funciones render_*
  template<typename RenderFunc>
  static bool register_renderer(const std::string& type_name, RenderFunc func) {
    auto& table = instance();
    RendererFn wrapper = RendererRegistry::make_renderer(func);
    return table.map.emplace(type_name, std::move(wrapper)).second;
  }
  // Versión original para RendererFn directo
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

inline void fill_metadata(const Core::EnvelopeMetadata& metadata, RenderContext& context) {
  context.stream().log(context.make_child_path("generic/metadata/topic"), rerun::TextLog(metadata.topic));
  context.stream().log(context.make_child_path("generic/metadata/typeName"), rerun::TextLog(metadata.typeName));
  context.stream().log(context.make_child_path("generic/metadata/schemaPath"), rerun::TextLog(metadata.schemaPath));
  context.stream().log(context.make_child_path("generic/metadata/timestampUsec"), rerun::TextLog(std::to_string(metadata.timestampUsec)));
}

inline void render_generic(const Core::EnvelopeMetadata& metadata,
                    ::capnp::AnyPointer::Reader payload,
                    RenderContext& context,
                    ::capnp::StructSchema schema) {
  fill_metadata(metadata, context);

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
// Header functions
// =====================
template<typename T, typename ReaderT>
void render_header(const Core::EnvelopeMetadata& metadata,
                  ReaderT reader_or_any,
                  RenderContext& context) {
  auto reader = resolve_reader<T>(reader_or_any);
  context.stream().log(context.make_child_path("header/seq"), rerun::Scalars(static_cast<double>(reader.getSeq())));
  context.stream().log(context.make_child_path("header/stampSec"), rerun::Scalars(static_cast<double>(reader.getStampSec())));
  context.stream().log(context.make_child_path("header/stampNsec"), rerun::Scalars(static_cast<double>(reader.getStampNsec())));
  context.stream().log(context.make_child_path("header/frameId"), rerun::TextLog(to_std_string(reader.getFrameId())));
}

// =====================
// Point functions
// =====================
template<typename T, typename ReaderT>
void render_point(const Core::EnvelopeMetadata& metadata,
                         ReaderT reader_or_any,
                         RenderContext& context) {
  auto reader = resolve_reader<T>(reader_or_any);
  
  const float x = static_cast<float>(reader.getX());
  const float y = static_cast<float>(reader.getY());
  const float z = static_cast<float>(reader.getZ());
  std::vector<rerun::Position3D> positions = {{x, y, z}};
  context.stream().log(context.make_child_path("point"), 
                       rerun::Points3D(positions).with_radii({0.05F}));
}

// Ejemplo de render que reutiliza otros
inline void render_point_stamped(const Core::EnvelopeMetadata& metadata,
                                ::capnp::AnyPointer::Reader payload,
                                RenderContext& context) {
  (void)metadata;
  auto reader = payload.getAs<::PointStamped>();
  {
    RenderContext subcontext(context.stream(), context.make_child_path("point_stamped"), context.logger());
    render_header<::Header, ::Header::Reader>(metadata, reader.getHeader(), subcontext);
  }
  {
    RenderContext subcontext(context.stream(), context.make_child_path("point_stamped"), context.logger());
    render_point<::Point, ::Point::Reader>(metadata, reader.getPoint(), subcontext);
  }
}

// =====================
// Vector3 functions
// =====================
template<typename T, typename ReaderT>
void render_geom_vector3(const Core::EnvelopeMetadata& metadata,
                        ReaderT reader_or_any,
                        RenderContext& context) {
  auto reader = resolve_reader<T>(reader_or_any);
  context.stream().log(context.make_child_path("vector3/x"), rerun::Scalars(reader.getX()));
  context.stream().log(context.make_child_path("vector3/y"), rerun::Scalars(reader.getY()));
  context.stream().log(context.make_child_path("vector3/z"), rerun::Scalars(reader.getZ()));
}

// =====================
// ColorRGBA functions
// =====================
template<typename T, typename ReaderT>
void render_color_rgba(const Core::EnvelopeMetadata& metadata,
                      ReaderT reader_or_any,
                      RenderContext& context) {
  auto reader = resolve_reader<T>(reader_or_any);
  std::ostringstream color_str;
  color_str << "rgba(" << reader.getR() << ", " << reader.getG() << ", " << reader.getB() << ", " << reader.getA() << ")";
  context.stream().log(context.make_child_path("color/value"), rerun::TextLog(color_str.str()));
  context.stream().log(context.make_child_path("color/r"), rerun::Scalars(static_cast<double>(reader.getR())));
  context.stream().log(context.make_child_path("color/g"), rerun::Scalars(static_cast<double>(reader.getG())));
  context.stream().log(context.make_child_path("color/b"), rerun::Scalars(static_cast<double>(reader.getB())));
  context.stream().log(context.make_child_path("color/a"), rerun::Scalars(static_cast<double>(reader.getA())));
}

// =====================
// Quaternion functions
// =====================
template<typename T, typename ReaderT>
void render_quaternion(const Core::EnvelopeMetadata& metadata, ReaderT reader_or_any, RenderContext& context) {
  auto reader = resolve_reader<T>(reader_or_any);
  const std::string parent_path = "quaternion";
  context.stream().log(context.make_child_path(parent_path + "/x"), rerun::Scalars(reader.getX()));
  context.stream().log(context.make_child_path(parent_path + "/y"), rerun::Scalars(reader.getY()));
  context.stream().log(context.make_child_path(parent_path + "/z"), rerun::Scalars(reader.getZ()));
  context.stream().log(context.make_child_path(parent_path + "/w"), rerun::Scalars(reader.getW()));
}

// =====================
// Pose functions
// =====================
template<typename T, typename ReaderT>
void render_pose(const Core::EnvelopeMetadata& metadata, ReaderT reader_or_any, RenderContext& context) {
  auto reader = resolve_reader<T>(reader_or_any);
  const std::string parent_path = "pose";
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/position"), context.logger());
    render_point<::Point, ::Point::Reader>(metadata, reader.getPosition(), subcontext);
  }
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/orientation"), context.logger());
    render_quaternion<::Quaternion, ::Quaternion::Reader>(metadata, reader.getOrientation(), subcontext);
  }
}

inline void render_pose_stamped(const Core::EnvelopeMetadata& metadata, ANY_READER payload, RenderContext& context) {
  auto reader = payload.getAs<::PoseStamped>();
  const std::string parent_path = "pose_stamped";
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/header"), context.logger());
    render_header<::Header, ::Header::Reader>(metadata, reader.getHeader(), subcontext);
  }
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/pose"), context.logger());
    render_pose<::Pose, ::Pose::Reader>(metadata, reader.getPose(), subcontext);
  }
}

template<typename T, typename ReaderT>
void render_pose_with_covariance(const Core::EnvelopeMetadata& metadata, ReaderT reader_or_any, RenderContext& context) {
  auto reader = resolve_reader<T>(reader_or_any);
  const std::string parent_path = "pose_with_covariance";
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/pose"), context.logger());
    render_pose<::Pose, ::Pose::Reader>(metadata, reader.getPose(), subcontext);
  }
  {
    auto cov = reader.getCovariance();
    std::vector<double> values(cov.size());
    for (size_t i = 0; i < cov.size(); ++i) values[i] = cov[i];
    context.stream().log(context.make_child_path(parent_path + "/covariance"), rerun::Scalars(values));
  }
}

// =====================
// Twist functions
// =====================
template<typename T, typename ReaderT>
void render_twist(const Core::EnvelopeMetadata& metadata, ReaderT reader_or_any, RenderContext& context) {
  auto reader = resolve_reader<T>(reader_or_any);
  const std::string parent_path = "twist";
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/linear"), context.logger());
    render_geom_vector3<::Vector3, ::Vector3::Reader>(metadata, reader.getLinear(), subcontext);
  }
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/angular"), context.logger());
    render_geom_vector3<::Vector3, ::Vector3::Reader>(metadata, reader.getAngular(), subcontext);
  }
}

template<typename T, typename ReaderT>
void render_twist_with_covariance(const Core::EnvelopeMetadata& metadata, ReaderT reader_or_any, RenderContext& context) {
  auto reader = resolve_reader<T>(reader_or_any);
  const std::string parent_path = "twist_with_covariance";
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/twist"), context.logger());
    render_twist<::Twist, ::Twist::Reader>(metadata, reader.getTwist(), subcontext);
  }
  {
    auto cov = reader.getCovariance();
    std::vector<double> values(cov.size());
    for (size_t i = 0; i < cov.size(); ++i) values[i] = cov[i];
    context.stream().log(context.make_child_path(parent_path + "/covariance"), rerun::Scalars(values));
  }
}

// =====================
// Transform functions
// =====================
template<typename T, typename ReaderT>
void render_transform(const Core::EnvelopeMetadata& metadata, ReaderT reader_or_any, RenderContext& context) {
  auto reader = resolve_reader<T>(reader_or_any);
  const std::string parent_path = "transform";
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/translation"), context.logger());
    render_geom_vector3<::Vector3, ::Vector3::Reader>(metadata, reader.getTranslation(), subcontext);
  }
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/rotation"), context.logger());
    render_quaternion<::Quaternion, ::Quaternion::Reader>(metadata, reader.getRotation(), subcontext);
  }
}

inline void render_transform_stamped(const Core::EnvelopeMetadata& metadata, ANY_READER payload, RenderContext& context) {
  auto reader = payload.getAs<::TransformStamped>();
  const std::string parent_path = "transform_stamped";
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/header"), context.logger());
    render_header<::Header, ::Header::Reader>(metadata, reader.getHeader(), subcontext);
  }
  {
    RenderContext subcontext(context.stream(), context.make_child_path(parent_path + "/transform"), context.logger());
    render_transform<::Transform, ::Transform::Reader>(metadata, reader.getTransform(), subcontext);
  }
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
  const auto color = marker.getColor();

  const float radius = compute_marker_radius(marker);
  std::vector<rerun::Position3D> positions = {{px, py, pz}};
  context.stream().log(context.make_child_path(base_segment + "/position"),
                       rerun::Points3D(positions)
                       .with_radii({radius})
                       .with_colors(rerun::components::Color(
                         static_cast<uint8_t>(color.getR() * 255),
                         static_cast<uint8_t>(color.getG() * 255),
                         static_cast<uint8_t>(color.getB() * 255),
                         static_cast<uint8_t>(color.getA() * 255)
                       )));
  // TODO: Darle un uso a "lifetime" o quitarlo
}

template<typename T, typename ReaderT>
void render_marker(const Core::EnvelopeMetadata& metadata,
                  ReaderT reader_or_any,
                  RenderContext& context) {
  auto marker = resolve_reader<T>(reader_or_any);
  const std::string marker_ns = to_std_string(marker.getNs());
  std::string segment = "markers/" +
                        (marker_ns.empty() ? std::string("default")
                                           : marker_ns) +
                        "_" + std::to_string(marker.getId());
  log_marker_details(marker, context, segment);
}

template<typename T, typename ReaderT>
void render_marker_array(const Core::EnvelopeMetadata& metadata,
                        ReaderT reader_or_any,
                        RenderContext& context) {
  auto array_reader = resolve_reader<T>(reader_or_any);
  auto markers = array_reader.getMarkers();
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
// Path functions
// =====================
inline void render_nav_path(const Core::EnvelopeMetadata& metadata,
                            ::capnp::AnyPointer::Reader payload,
                            RenderContext& context) {
  auto reader = payload.getAs<::Path>();
  // Render header
  {
    RenderContext subcontext(context.stream(), context.make_child_path("path"), context.logger());
    render_header<::Header, ::Header::Reader>(metadata, reader.getHeader(), subcontext);
  }
  // Render poses as a line connecting the points
  auto poses = reader.getPoses();
  std::vector<rerun::Position3D> points;
  for (auto pose : poses) {
    auto pos = pose.getPose().getPosition();
    points.emplace_back(pos.getX(), pos.getY(), -pos.getZ());
  }
    if (points.size() >= 2) {
      // Usar la API C++: strips, color verde, radio
      std::vector<rerun::components::LineStrip3D> strips;
      strips.emplace_back(points);
      std::vector<rerun::components::Color> colors = { rerun::components::Color(0, 255, 0) };
      std::vector<rerun::components::Radius> radii = { rerun::components::Radius(0.02f) };
      context.stream().log(
        context.make_child_path("path/line"),
        rerun::LineStrips3D(strips)
          .with_colors(colors)
          .with_radii(radii)
      );
    } else if (points.size() == 1) {
    context.stream().log(context.make_child_path("path/line"), rerun::Points3D(points).with_radii({0.04f}));
  } else {
    context.stream().log(context.make_child_path("path/line"), rerun::TextLog("(empty path: no points)"));
  }
}

// =====================
// Odometry functions
// =====================
inline void render_odometry(const Core::EnvelopeMetadata& metadata,
                     ::capnp::AnyPointer::Reader payload,
                     RenderContext& context) {
  (void)metadata;
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

// =====================
// PointCloud functions
// =====================
inline void log_map(rerun::RecordingStream& rec,
           Core::Logger* logger,
           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
           std::string index = "") {
  auto width = cloud->width;
  auto height = cloud->height;
  size_t num_points = cloud->points.size();
  if (logger)
    logger->debug("Received chunk with %zu points", num_points);
  if (num_points == 0) return;

  std::vector<rerun::Position3D> positions;
  std::vector<rerun::Color> colors;

  positions.reserve(num_points);
  colors.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    auto point = cloud->points[i];
    float x = point.x;
    float y = point.y;
    float z = point.z;
    positions.emplace_back(x, y, z);
    colors.emplace_back(rerun::Color(point.r, point.g, point.b));
  }

  if (index.size() > 0) {
    index += "/";
  }
  // Log to Rerun
  rec.log("world/map/" + index,
    rerun::Points3D(positions).with_colors(colors));

  // Log statistics
  rec.log("stats/map/" + index + "point_count",
    rerun::Scalars(static_cast<double>(positions.size())));
  rec.log("stats/map/" + index + "total_received",
    rerun::Scalars(static_cast<double>(num_points)));
  rec.log("stats/map/" + index + "image_dimensions",
    rerun::TextLog("Dimensions: " + std::to_string(width) + "x" + std::to_string(height)));
}

inline void render_pointcloud(const Core::EnvelopeMetadata& metadata,
                            ::capnp::AnyPointer::Reader payload,
                            RenderContext& context) {
  (void)metadata;
  auto reader = payload.getAs<::PointCloud>();
  auto data_reader = reader.getData();
  auto width = reader.getWidth();
  auto height = reader.getHeight();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>(width, height));

  std::stringstream buffer(
      std::string((char *)data_reader.begin(), data_reader.size()));

  static pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> decoder;
  try {
    decoder.decodePointCloud(buffer, cloud);
  } catch (const std::exception &e) {
    if (context.logger())
      context.logger()->warn("Error while decoding cloudpoint: %s", e.what());
      context.stream().log(context.make_child_path("pointcloud/error"), rerun::TextLog(e.what()));
    return;
  }

  // Usar la función utilitaria para loguear igual que Viewer::log_map
  log_map(context.stream(), context.logger(), cloud);
}

// =================================
// Template para renders reusables
// =================================
// [[maybe_unused]] const bool point_registered = 
//  RendererRegistry::register_renderer("Point",
//                                      &render_point<::Point, ANY_READER>);
// ======================================
// Template para renders de un solo uso
// =================================
// [[maybe_unused]] const bool odometry_registered =
//   RendererRegistry::register_renderer("Odometry", &render_odometry);

// =====================
// std_msgs registrations
// =====================
[[maybe_unused]] const bool header_registered =
  RendererRegistry::register_renderer("Header", 
                                      &render_header<::Header, ANY_READER>);
[[maybe_unused]] const bool color_rgba_registered =
  RendererRegistry::register_renderer("ColorRGBA", 
                                      &render_color_rgba<::ColorRGBA, ANY_READER>);
// =====================
// geometry_msgs registrations
// =====================
[[maybe_unused]] const bool vector3_registered =
  RendererRegistry::register_renderer("Vector3", 
                                      &render_geom_vector3<::Vector3, ANY_READER>);
[[maybe_unused]] const bool point_registered = 
RendererRegistry::register_renderer("Point",
                                    &render_point<::Point, ANY_READER>);
[[maybe_unused]] const bool point_stamped_registered =
  RendererRegistry::register_renderer("PointStamped", &render_point_stamped);
[[maybe_unused]] const bool quaternion_registered =
  RendererRegistry::register_renderer("Quaternion", 
                                      &render_quaternion<::Quaternion, ANY_READER>);
[[maybe_unused]] const bool pose_registered =
  RendererRegistry::register_renderer("Pose", 
                                      &render_pose<::Pose, ANY_READER>);
[[maybe_unused]] const bool pose_with_cov_registered =
  RendererRegistry::register_renderer("PoseWithCovariance", 
                                      &render_pose_with_covariance<::PoseWithCovariance, ANY_READER>);
[[maybe_unused]] const bool twist_registered =
  RendererRegistry::register_renderer("Twist", 
                                      &render_twist<::Twist, ANY_READER>);
[[maybe_unused]] const bool twist_with_cov_registered =
  RendererRegistry::register_renderer("TwistWithCovariance", 
                                      &render_twist_with_covariance<::TwistWithCovariance, ANY_READER>);
[[maybe_unused]] const bool pose_stamped_registered =
  RendererRegistry::register_renderer("PoseStamped", &render_pose_stamped);
[[maybe_unused]] const bool transform_registered =
  RendererRegistry::register_renderer("Transform", 
                                      &render_transform<::Transform, ANY_READER>);
[[maybe_unused]] const bool transform_stamped_registered =
  RendererRegistry::register_renderer("TransformStamped", &render_transform_stamped);

// =====================
// nav_msgs registrations
// =====================
[[maybe_unused]] const bool nav_path_registered =
  RendererRegistry::register_renderer("Path", &render_nav_path);
[[maybe_unused]] const bool odometry_registered =
  RendererRegistry::register_renderer("Odometry", &render_odometry);

// =====================
// nav_msgs registrations
// =====================
[[maybe_unused]] const bool marker_registered =
  RendererRegistry::register_renderer("Marker", 
                                      &render_marker<::Marker, ANY_READER>);
[[maybe_unused]] const bool marker_array_registered =
  RendererRegistry::register_renderer("MarkerArray", 
                                      &render_marker_array<::MarkerArray, ANY_READER>);

[[maybe_unused]] const bool pointcloud_registered =
  RendererRegistry::register_renderer("PointCloud", &render_pointcloud);
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

class Renderer {
public:
  Renderer(Viewer* viewer)
      : viewer_(viewer),
        stream_(viewer_->get_recording_stream()),
        shutting_down_(false),
        ignored_prefixes_({"registry/"}) {
    topics_client_ = viewer_->public_create_action_client<TopicsListRequest, TopicsListResponse>(
        "registry/topics");
    std::string whitelist_arg = viewer_->get_argument<std::string>("--whitelist");
    std::string blacklist_arg = viewer_->get_argument<std::string>("--blacklist");
    if (!whitelist_arg.empty()) {
      set_topic_whitelist(whitelist_arg);
    }
    if (!blacklist_arg.empty()) {
      set_topic_blacklist(blacklist_arg);
    }
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
  // Methods to configure whitelist and blacklist from string
  void set_topic_whitelist(const std::string& whitelist_str) {
    topic_whitelist_.clear();
    std::istringstream ss(whitelist_str);
    std::string item;
    while (std::getline(ss, item, ',')) {
      if (!item.empty()) topic_whitelist_.insert(item);
    }
  }
  void set_topic_blacklist(const std::string& blacklist_str) {
    topic_blacklist_.clear();
    std::istringstream ss(blacklist_str);
    std::string item;
    while (std::getline(ss, item, ',')) {
      if (!item.empty()) topic_blacklist_.insert(item);
    }
  }

private:
  Viewer* viewer_;
  std::unordered_set<std::string> topic_whitelist_;
  std::unordered_set<std::string> topic_blacklist_;

  void topic_refresh_loop() {
    while (!shutting_down_.load()) {
      try {
        refresh_topics();
      } catch (const std::exception& ex) {
        viewer_->get_logger()->warn("Topic refresh failed: %s", ex.what());
      }
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }
  }

  void refresh_topics() {
    if (!topics_client_) {
      return;
    }

    auto request = topics_client_->new_msg();
    request.content.setIncludeInternal(false);
    auto response = request.send();
    if (!response.has_value()) {
      viewer_->get_logger()->debug("Topics action returned no response");
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
      // TODO: Filtrar por "TypeName"
      if (topic_blacklist_.count(topic_name) > 0) {
        continue;
      }
      if (!topic_whitelist_.empty() && topic_whitelist_.count(topic_name) == 0) {
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
      viewer_->get_logger()->info("Unsubscribed from topic %s", entry.first.c_str());
    }
  }

  void ensure_subscription(const std::string& topic_name) {
    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      if (topic_subscribers_.find(topic_name) != topic_subscribers_.end()) {
        return;
      }
    }

    auto subscriber = viewer_->public_create_subscriber<::capnp::AnyPointer>(
        topic_name,
        [this, topic_name](const Core::IncomingMessage<::capnp::AnyPointer>& msg) {
          handle_any_message(topic_name, msg);
        });
    subscriber->set_loglevel(viewer_->get_logger()->get_level());

    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      topic_subscribers_.emplace(topic_name, subscriber);
    }
    viewer_->get_logger()->info("Subscribed to topic %s", topic_name.c_str());
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
    const std::string& topic_name = msg.metadata.topic.empty() ? default_topic : msg.metadata.topic;
    const capnp::StructSchema& struct_schema = schema_cache_.resolve(
      msg.metadata.schemaPath, msg.metadata.typeName, *viewer_->get_logger());

    RerunRenderers::RenderContext context(*stream_, topic_name, viewer_->get_logger());
    RerunRenderers::dispatch_message(msg.metadata, msg.getRawPayload(),
                     context, struct_schema);
  }

  std::shared_ptr<rerun::RecordingStream> stream_;
  std::shared_ptr<Core::ActionClient<TopicsListRequest, TopicsListResponse>> topics_client_;
  std::unordered_map<std::string, std::shared_ptr<Core::Subscriber<::capnp::AnyPointer>>> topic_subscribers_;
  std::mutex subscribers_mutex_;
  SchemaCache schema_cache_;
  std::thread refresh_thread_;
  std::atomic_bool shutting_down_;
  std::vector<std::string> ignored_prefixes_;
};

}  // namespace RegistryExample

#endif  // RERUN_RENDER_REGISTRY_HPP

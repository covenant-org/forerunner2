#ifndef RERUN_RENDER_REGISTRY_HPP
#define RERUN_RENDER_REGISTRY_HPP

#include <capnp/any.h>
#include <capnp/schema.h>
#include <rerun/recording_stream.hpp>

#include "../core/message.hpp"

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <unordered_map>
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
  static bool register_renderer(const std::string& type_name, RendererFn fn);
  static std::optional<RendererFn> find(const std::string& type_name);

 private:
  RendererRegistry() = default;
  static RendererRegistry& instance();

  std::unordered_map<std::string, RendererFn> map;
};

void dispatch_message(const Core::EnvelopeMetadata& metadata,
                      ::capnp::AnyPointer::Reader payload,
                      RenderContext& context,
                      ::capnp::StructSchema schema);

template <typename T>
void dispatch_message(const Core::IncomingMessage<T>& msg,
                      RenderContext& context) {
  const ::capnp::StructSchema schema = ::capnp::Schema::from<T>().asStruct();
  dispatch_message(msg.metadata, msg.getRawPayload(), context, schema);
}

}  // namespace RerunRenderers

#endif  // RERUN_RENDER_REGISTRY_HPP

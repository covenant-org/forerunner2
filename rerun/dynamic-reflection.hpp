#ifndef RERUN_DYNAMIC_REFLECTION_HPP
#define RERUN_DYNAMIC_REFLECTION_HPP

#include "vertex.hpp"
#include <capnp/any.h>
#include <capnp/dynamic.h>
#include <capnp/schema-parser.h>
#include <memory>
#include <string>

class DynamicReflection : public Core::Vertex {
 public:
    explicit DynamicReflection(Core::ArgumentParser args);

    void run() override;

 private:
   void message_cb(const Core::IncomingMessage<capnp::AnyPointer>& msg);

    std::string _schema_path;
    std::string _type_name;
    std::string _topic;
    capnp::StructSchema _schema;
   std::shared_ptr<Core::Subscriber<capnp::AnyPointer>> _topic_sub;
   capnp::SchemaParser _parser;
};

#endif  // RERUN_DYNAMIC_REFLECTION_HPP

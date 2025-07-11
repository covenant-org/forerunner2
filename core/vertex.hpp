#include <string>
#include <map>

#ifndef VERTEX_HPP
#define VERTEX_HPP

namespace Core {
class Vertex {
 private:
    static std::map<std::string, std::string> _args;

    void parse_args(int, char**);
 public:
    Vertex(int, char **);
};
};  // namespace Core

#endif  // VERTEX_HPP
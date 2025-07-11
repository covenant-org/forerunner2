#include "vertex.hpp"

namespace Core {
Vertex::Vertex(char **args) {
    this->parse_args(args);
}
};  // namespace Core
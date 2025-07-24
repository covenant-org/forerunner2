#ifndef ARGUMENT_PARSER_HPP
#define ARGUMENT_PARSER_HPP

#include <argparse/argparse.hpp>
#include <memory>

namespace Core {
class Vertex;

class ArgumentParser {
  friend class Vertex;

 private:
  std::shared_ptr<argparse::ArgumentParser> _program;
  int _argc;
  char **_argv;

  std::string get_program_name() {
    auto program_name = std::string(_argv[0]);
    auto last_seg = program_name.find_last_of('/');
    if (last_seg != std::variant_npos) {
      program_name = std::string(program_name.data() + last_seg + 1);
    }
    return program_name;
  }

 protected:
  std::string _program_name;
  void parse() { _program->parse_args(_argc, _argv); }

 public:
  ArgumentParser(int argc, char **argv) : _argc(argc), _argv(argv) {
    _program_name = get_program_name();
    _program = std::make_shared<argparse::ArgumentParser>(_program_name);
  }

  template <typename... Targs>
  argparse::Argument &add_argument(Targs... f_args) {
    return _program->add_argument(f_args...);
  }
  template <typename T = std::string>
  T get_argument(std::string_view arg_name) const {
    return _program->get<T>(arg_name);
  }
};
}  // namespace Core

#endif

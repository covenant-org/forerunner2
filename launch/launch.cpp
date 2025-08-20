#include "launch.hpp"
#include "utils.hpp"

std::map<std::string, std::string> Launch::find_executable_files(
    const std::filesystem::path& dir,
    const std::vector<std::string>& exclude_folders) {
  std::map<std::string, std::string> exe_map;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (entry.is_regular_file()) {
      // Excluir carpetas
      if (!exclude_folders.empty()) {
        bool in_excluded = false;
        auto rel = std::filesystem::relative(entry.path(), dir);
        for (const auto& folder : exclude_folders) {
          if (rel.string().find(folder + "/") == 0 || rel.string() == folder) {
            in_excluded = true;
            break;
          }
        }
        if (in_excluded) continue;
      }
      // Filtrar archivos con extensión
      if (entry.path().has_extension()) continue;

      // Verificar permisos de ejecución
      auto perms = entry.status().permissions();
      if ((perms & std::filesystem::perms::owner_exec) !=
              std::filesystem::perms::none ||
          (perms & std::filesystem::perms::group_exec) !=
              std::filesystem::perms::none ||
          (perms & std::filesystem::perms::others_exec) !=
              std::filesystem::perms::none) {
        std::string exe_name = entry.path().filename().string();
        exe_map[exe_name] = entry.path().string();
      }
    }
  }
  return exe_map;
}

Launch::Launch(argparse::ArgumentParser& parser,
               const std::vector<std::string>& exclude,
               const std::vector<std::string>& names,
               const std::vector<std::vector<std::string>>& args,
               double delay_seconds)
    : _exclude_folders(exclude), _delay_seconds(delay_seconds) {
  int registry_port = parser.get<int>("--registry-port");
  int registry_threads = parser.get<int>("--registry-threads");
  _log_level = parser.get<std::string>("--log-level");

  _logger.set_classname("launch");
  _logger.set_level(string_to_loglevel(_log_level));

  _root_path = Core::find_root(".root", 10);
  if (_root_path.empty()) {
    _logger.error("Root path not found.");
    return;
  }
  executables = find_executable_files(_root_path, _exclude_folders);

  all_names = names;
  all_args = args;
  std::vector<std::string> registry_args;
  if (registry_port != 0 && registry_threads != 0) {
    registry_args = {"--port", std::to_string(registry_port), "--threads",
                     std::to_string(registry_threads)};
  }
  all_names.insert(all_names.begin(), "registry");
  all_args.insert(all_args.begin(), registry_args);
}

Launch::Launch(argparse::ArgumentParser& parser,
               const std::vector<std::string>& names,
               const std::vector<std::vector<std::string>>& args,
               double delay_seconds)
    : Launch(parser, default_exclude_folders, names, args, delay_seconds) {}

// Constructor que recibe un NodesYamlParser

Launch::Launch(argparse::ArgumentParser& parser,
               const NodesYamlParser& yaml_parser,
               double delay_seconds)
    : Launch(parser, default_exclude_folders, yaml_parser.get_executables(),
             [&parser, &yaml_parser]() {
               std::vector<std::vector<std::string>> args;
               std::string log_level = parser.get<std::string>("--log-level");
               for (const auto& name : yaml_parser.get_executables()) {
                 std::vector<std::string> exe_args =
                     yaml_parser.get_args_line(name);
                 exe_args.insert(exe_args.begin(), log_level);
                 exe_args.insert(exe_args.begin(), "--log-level");
                 args.push_back(exe_args);
               }
               return args;
             }(), delay_seconds) {}

std::map<std::string, std::string> Launch::get_executables() {
  return executables;
}

void Launch::run_executables() {
  run_executables({}, {});
}

int Launch::run_executable(const std::string& name,
                           const std::vector<std::string>& args) {
  auto it = executables.find(name);
  if (it == executables.end()) {
    _logger.error("Executable not found: %s", name.c_str());
    return -1;
  }
  try {
    _logger.info("Starting %s at %s", name.c_str(), it->second.c_str());
    std::stringstream command;
    command << it->second.c_str();
    if (!args.empty()) {
      for (const std::string& arg : args) {
        command << " " << arg;
      }
    }
    return system(command.str().c_str());
  } catch (const std::exception& e) {
    _logger.error("Error running %s: %s", name.c_str(), e.what());
    return -1;
  }
}

void Launch::run_executables(
    const std::vector<std::string>& names,
    const std::vector<std::vector<std::string>>& arguments) {
  const std::vector<std::string>& use_names = names.empty() ? all_names : names;
  const std::vector<std::vector<std::string>>& use_args = arguments.empty() ? all_args : arguments;
  std::vector<std::thread> threads;
  for (size_t i = 0; i < use_names.size(); ++i) {
    std::vector<std::string> args;
    if (i < use_args.size()) {
      args = use_args[i];
    }
    _logger.debug("Launching process [%zu]: %s", i, use_names[i].c_str());
    threads.emplace_back(
        [this, name = use_names[i], args]() { run_executable(name, args); });
    if (_delay_seconds > 0.0 && i + 1 < use_names.size()) {
      std::this_thread::sleep_for(std::chrono::duration<double>(_delay_seconds));
    }
  }
  for (auto& t : threads) {
    if (t.joinable()) t.join();
  }
}

void print_argparse_help(const argparse::ArgumentParser& parser) {
  Core::Logger logger(Core::LogLevel::INFO, "application.log", "launch");
  std::stringstream ss;
  ss << parser;
  logger.info("\n%s", ss.str().c_str());
}

Core::LogLevel Launch::string_to_loglevel(const std::string& level) {
    if (level == "debug") return Core::LogLevel::DEBUG;
    if (level == "info") return Core::LogLevel::INFO;
    if (level == "warn") return Core::LogLevel::WARN;
    if (level == "error") return Core::LogLevel::ERROR;
    return Core::LogLevel::INFO; // valor por defecto
}

int main(int argc, char** argv) {
  argparse::ArgumentParser parser;
  parser.add_argument("--yaml-path")
      .required()
      .help("Path to the YAML configuration file");
  parser.add_argument("--registry-port")
      .default_value(0)
      .help("Registry port (optional, requires --registry-threads)")
      .nargs(1);
  parser.add_argument("--registry-threads")
      .default_value(0)
      .help(
          "Number of threads for the registry (optional, requires "
          "--registry-port)")
      .nargs(1);
  parser.add_argument("--log-level")
      .default_value("info")
      .help("Log level (optional)")
      .nargs(1);
  parser.add_argument("--delay-seconds")
      .default_value(0.0)
      .help("Delay (in seconds, decimal allowed) between starting each process")
      .nargs(1);

  try {
    parser.parse_args(argc, argv);

    int registry_port = parser.get<int>("--registry-port");
    int registry_threads = parser.get<int>("--registry-threads");

    // Validación lógica adicional
    if ((registry_port != 0 && registry_threads == 0) ||
        (registry_threads != 0 && registry_port == 0)) {
      print_argparse_help(parser);
      return 1;
    }

  } catch (const std::runtime_error& err) {
    print_argparse_help(parser);
    return 1;
  }

  std::string yaml_path = parser.get<std::string>("--yaml-path");
  double delay_seconds = 0.0;
  try {
    delay_seconds = std::stod(parser.get<std::string>("--delay-seconds"));
  } catch (const std::exception&) {
    delay_seconds = 0.0;
  }
  NodesYamlParser yaml_parser(yaml_path);
  Launch launch_instance(parser, yaml_parser, delay_seconds);
  
  launch_instance.run_executables();
  return 0;
}

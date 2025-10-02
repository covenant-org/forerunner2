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
      bool is_executable = (perms & (std::filesystem::perms::owner_exec |
                                     std::filesystem::perms::group_exec |
                                     std::filesystem::perms::others_exec)) != 
                           std::filesystem::perms::none;
      
      if (is_executable) {
        std::string exe_name = entry.path().filename().string();
        exe_map[exe_name] = entry.path().string();
      }
    }
  }
  return exe_map;
}

std::vector<std::vector<std::string>> Launch::build_args_from_yaml(
    argparse::ArgumentParser& parser, const NodesYamlParser& yaml_parser) {
  std::vector<std::vector<std::string>> args;
  std::string log_level = parser.get<std::string>("--log-level");
  const auto& names = yaml_parser.get_executables();
  
  // Count occurrences of each executable name from YAML only
  std::map<std::string, int> name_counts;
  for (const auto& name : names) {
    name_counts[name]++;
  }
  
  // Create instance counters for duplicates
  std::map<std::string, int> instance_counters;
  
  for (size_t i = 0; i < names.size(); ++i) {
    size_t idx = args.size();
    std::vector<std::string> exe_args = yaml_parser.get_args_line(idx);
    
    const std::string& exe_name = names[i];
    
    // Only add instance-id if this executable appears more than once in YAML
    if (name_counts[exe_name] > 1) {
      std::string instance_id = std::to_string(instance_counters[exe_name]);
      instance_counters[exe_name]++;
      exe_args.insert(exe_args.begin(), instance_id);
      exe_args.insert(exe_args.begin(), "--instance-id");
    }
    
    exe_args.insert(exe_args.begin(), log_level);
    exe_args.insert(exe_args.begin(), "--log-level");
    args.push_back(exe_args);
  }
  return args;
}

std::vector<std::string> Launch::build_registry_args(
    argparse::ArgumentParser& parser, const std::vector<std::string>& names) {
  int registry_port = parser.get<int>("--registry-port");
  int registry_threads = parser.get<int>("--registry-threads");
  std::string log_level = parser.get<std::string>("--log-level");
  
  std::vector<std::string> registry_args = {"--log-level", log_level};
  
  // Add port if specified
  if (registry_port != 0) {
    registry_args.push_back("--port");
    registry_args.push_back(std::to_string(registry_port));
  }
  
  // Add threads if specified
  if (registry_threads != 0) {
    registry_args.push_back("--threads");
    registry_args.push_back(std::to_string(registry_threads));
  }
  
  // Count registry instances to determine if we need instance-id
  int registry_count = 0;
  for (const auto& name : names) {
    if (name == "registry") registry_count++;
  }
  
  // Add instance-id if there are duplicate registries
  if (registry_count > 0) {
    registry_args.insert(registry_args.begin(), std::to_string(registry_count));
    registry_args.insert(registry_args.begin(), "--instance-id");
  }
  
  return registry_args;
}

Launch::Launch(argparse::ArgumentParser& parser,
               const std::vector<std::string>& exclude,
               const std::vector<std::string>& names,
               const std::vector<std::vector<std::string>>& args)
    : _exclude_folders(exclude) {
  
  // Initialize logger and basic setup
  _log_level = parser.get<std::string>("--log-level");
  _logger.set_classname("launch");
  _logger.set_level(Core::Logger::parse_level(_log_level));
  
  // Find root path
  _root_path = Core::find_root(".root", 10);
  if (_root_path.empty()) {
    _logger.error("Root path not found.");
    return;
  }
  
  executables = find_executable_files(_root_path, _exclude_folders);

  // Prepare final lists with registry always first
  std::vector<std::string> final_names = {"registry"};
  std::vector<std::vector<std::string>> final_args = {build_registry_args(parser, names)};
  
  // Add user-provided executables
  final_names.insert(final_names.end(), names.begin(), names.end());
  final_args.insert(final_args.end(), args.begin(), args.end());
  
  if (!final_names.empty()) {
    run_executables(final_names, final_args);
  }
}

Launch::Launch(argparse::ArgumentParser& parser,
               const std::vector<std::string>& names,
               const std::vector<std::vector<std::string>>& args)
    : Launch(parser, default_exclude_folders, names, args) {}

// Constructor que recibe un NodesYamlParser
Launch::Launch(argparse::ArgumentParser& parser,
               const NodesYamlParser& yaml_parser)
    : Launch(parser, default_exclude_folders, yaml_parser.get_executables(),
             build_args_from_yaml(parser, yaml_parser)) {}

std::map<std::string, std::string> Launch::get_executables() {
  return executables;
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
    
    // Build command string
    std::string command = it->second;
    for (const std::string& arg : args) {
      command += " " + arg;
    }
    
    // Debug: log the full command that will be executed
    _logger.debug("[LAUNCH] exec command: %s", command.c_str());
    return system(command.c_str());
  } catch (const std::exception& e) {
    _logger.error("Error running %s: %s", name.c_str(), e.what());
    return -1;
  }
}

void Launch::run_executables(
    const std::vector<std::string>& names,
    const std::vector<std::vector<std::string>>& arguments) {
  std::vector<std::thread> threads;
  for (size_t i = 0; i < names.size(); ++i) {
    std::vector<std::string> args;
    if (i < arguments.size()) {
      args = arguments[i];
    }
    threads.emplace_back(
        [this, name = names[i], args]() { run_executable(name, args); });
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

int main(int argc, char** argv) {
  argparse::ArgumentParser parser;
  parser.add_argument("--yaml-path")
      .required()
      .help("Path to the YAML configuration file");
  parser.add_argument("--registry-port")
      .default_value(0)
      .help("Registry port (optional)")
      .scan<'i', int>();
  parser.add_argument("--registry-threads")
      .default_value(0)
      .help("Number of threads for the registry (optional)")
      .scan<'i', int>();
  parser.add_argument("--log-level")
      .default_value("info")
      .help("Log level (optional)");

  try {
    parser.parse_args(argc, argv);

    int registry_port = parser.get<int>("--registry-port");
    int registry_threads = parser.get<int>("--registry-threads");

  } catch (const std::runtime_error& err) {
    print_argparse_help(parser);
    return 1;
  }

  std::string yaml_path = parser.get<std::string>("--yaml-path");
  NodesYamlParser yaml_parser(yaml_path);
  Launch launch_instance(parser, yaml_parser);
  return 0;
}

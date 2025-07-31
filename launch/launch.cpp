#include "launch.hpp"


std::string Launch::find_root(const std::string& filename, int max_levels) {
    std::filesystem::path current = std::filesystem::current_path();
    for (int i = 0; i <= max_levels; ++i) {
        for (const auto& entry : std::filesystem::directory_iterator(current)) {
            if (entry.is_regular_file() && entry.path().filename() == filename) {
                return entry.path().parent_path().string();
            }
        }
        if (current.has_parent_path()) {
            current = current.parent_path();
        } else {
            break;
        }
    }
    _logger.error("Root not found");
    return "";
}

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

            auto perms = entry.status().permissions();
            if ((perms & std::filesystem::perms::owner_exec) != std::filesystem::perms::none ||
                (perms & std::filesystem::perms::group_exec) != std::filesystem::perms::none ||
                (perms & std::filesystem::perms::others_exec) != std::filesystem::perms::none) {
                std::string exe_name = entry.path().filename().string();
                exe_map[exe_name] = entry.path().string();
            }
        }
    }
    return exe_map;
}

void Launch::set_log_level(Core::LogLevel level) {
    _logger.set_level(level);
}


Launch::Launch(argparse::ArgumentParser& parser, const std::vector<std::string>& exclude, const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& args)
    : _exclude_folders(exclude) {
    int registry_port = parser.get<int>("--registry-port");
    int registry_threads = parser.get<int>("--registry-threads");
    _root_path = find_root(".root", 10);
    if (_root_path.empty()) {
        _logger.error("Root path not found.");
        return;
    }
    executables = find_executable_files(_root_path, _exclude_folders);

    std::vector<std::string> all_names = names;
    std::vector<std::vector<std::string>> all_args = args;
    std::vector<std::string> registry_args;
    if (registry_port != 0 && registry_threads != 0) {
        registry_args = {"--port", std::to_string(registry_port), "--threads", std::to_string(registry_threads)};
    }
    all_names.insert(all_names.begin(), "registry");
    all_args.insert(all_args.begin(), registry_args);
    if (!all_names.empty()) {
        run_executables(all_names, all_args);
    }
}


Launch::Launch(argparse::ArgumentParser& parser, const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& args)
    : Launch(parser, default_exclude_folders, names, args) {}

// Constructor que recibe un NodesYamlParser

Launch::Launch(argparse::ArgumentParser& parser, const NodesYamlParser& yaml_parser)
    : Launch(parser, default_exclude_folders, yaml_parser.get_executables(), [&parser, &yaml_parser]() {
        std::vector<std::vector<std::string>> args;
        std::string log_level = parser.get<std::string>("--log-level");
        for (const auto& name : yaml_parser.get_executables()) {
            std::vector<std::string> exe_args = yaml_parser.get_args_line(name);
            exe_args.insert(exe_args.begin(), log_level);
            exe_args.insert(exe_args.begin(), "--log-level");
            args.push_back(exe_args);
        }
        return args;
    }()) {}



std::map<std::string, std::string> Launch::get_executables() {
    return executables;
}

int Launch::run_executable(const std::string& name, const std::vector<std::string>& args) {
    auto it = executables.find(name);
    if (it == executables.end()) {
        _logger.error("Executable not found: %s", name.c_str());
        return -1;
    }
    try {
        if (args.empty()) {
            boost::process::child c(it->second);
            c.wait();
            _logger.info("%s finished with code: %d", name.c_str(), c.exit_code());
            return c.exit_code();
        } else {
            boost::process::child c(it->second, boost::process::args(args));
            c.wait();
            _logger.info("%s finished with code: %d", name.c_str(), c.exit_code());
            return c.exit_code();
        }
    } catch (const std::exception& e) {
        _logger.error("Error running %s: %s", name.c_str(), e.what());
        return -1;
    }
}

void Launch::run_executables(const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& arguments) {
    std::vector<std::thread> threads;
    for (size_t i = 0; i < names.size(); ++i) {
        std::vector<std::string> args;
        if (i < arguments.size()) {
            args = arguments[i];
        }
        threads.emplace_back([this, name = names[i], args]() {
            run_executable(name, args)
        });
    }
    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }
}

void print_help() {
    Core::Logger logger(Core::LogLevel::INFO, "application.log", "launch");
    logger.info("\nUsage: launch [options]\n"
                "  --yaml-path <path>         Path to the YAML configuration file (required)\n"
                "  --registry-port <port>     Registry port (optional, requires --registry-threads)\n"
                "  --registry-threads <n>     Number of threads for the registry (optional, requires --registry-port)\n"
                "  --help, -h                 Show this help message\n"
                "\nNotes:\n  - --yaml-path is required.\n  - If you use --registry-port you must also use --registry-threads, and vice versa.\n");
}

int main(int argc, char** argv) {
    argparse::ArgumentParser parser;
    parser.add_argument("--yaml-path")
        .required()
        .help("Path to the YAML configuration file (required)");
    parser.add_argument("--registry-port")
        .default_value(0)
        .help("Registry port (optional, requires --registry-threads)")
        .nargs(1);
    parser.add_argument("--registry-threads")
        .default_value(0)
        .help("Number of threads for the registry (optional, requires --registry-port)")
        .nargs(1);
    parser.add_argument("--log-level")
        .default_value("debug")
        .help("Log level (optional)")
        .nargs(1);
    parser.add_argument("--help", "-h")
        .default_value(false)
        .implicit_value(true)
        .help("Show this help message");

    parser.parse_args(argc, argv);

    if (parser.get<bool>("--help")) {
        print_help();
        return 0;
    }

    std::string yaml_path = parser.get<std::string>("--yaml-path");
    int registry_port = parser.get<int>("--registry-port");
    int registry_threads = parser.get<int>("--registry-threads");

    // Validación de argumentos obligatorios y dependientes
    if (yaml_path.empty() ||
        ((registry_port != 0 && registry_threads == 0) || (registry_threads != 0 && registry_port == 0))) {
        print_help();
        return 1;
    }
    NodesYamlParser yaml_parser(yaml_path);
    Launch launch_instance(parser, yaml_parser);
    return 0;
}

// int main() {
//     int argc = 1;
//     char arg0[] = "dummy";
//     char* argv_fake[] = {arg0, nullptr};
//     Launch l(argc, argv_fake, std::vector<std::string>{}, std::vector<std::vector<std::string>>{});
//     const auto& exes = l.get_executables();
//     _logger.info("Executables encontrados:");
//     for (const auto& exe : exes) {
//         _logger.info("Nombre: {} | Ruta: {}", exe.first, exe.second);
//     }
//     return 0;
// }



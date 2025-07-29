
#include "launch.hpp"


std::string launch::find_root(const std::string& filename, int max_levels) {
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
    std::cout << "Root not found" << std::endl;
    return "";
}

std::map<std::string, std::string> launch::find_executable_files(
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

void launch::set_log_level(Core::LogLevel level) {
    _logger.set_level(level);
}



launch::launch(int argc, char** argv, const std::vector<std::string>& exclude, const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& args)
    : _args(argc, argv), _exclude_folders(exclude) {
    // Aquí deberías agregar argumentos a _args si es necesario
    // _args.add_argument(...)
    // _args.parse(); // parse es protected, así que solo puedes usar la API pública

    // Configuración de logger (ajusta según tu API pública)
    // _logger.set_classname(...); // Si tienes acceso público
    // auto level = _args.get_argument<Core::LogLevel>("--log-level");
    // _logger.set_level(level);

    _root_path = find_root(".root", 10);
    if (_root_path.empty()) {
        std::cerr << "Root path not found." << std::endl;
        return;
    }
    executables = find_executable_files(_root_path, _exclude_folders);
    if (!names.empty()) {
        run_executables(names, args);
    }
}

launch::launch(int argc, char** argv, const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& args)
    : launch(argc, argv, default_exclude_folders, names, args) {}

// Constructor que recibe un NodesYamlParser
launch::launch(int argc, char** argv, const NodesYamlParser& parser)
    : launch(argc, argv, default_exclude_folders,
        parser.get_executables(),
        [&parser]() {
            std::vector<std::vector<std::string>> args;
            for (const auto& name : parser.get_executables()) {
                args.push_back(parser.get_args_line(name));
            }
            return args;
        }()) {}



std::map<std::string, std::string> launch::get_executables() {
    return executables;
}

int launch::run_executable(const std::string& name, const std::vector<std::string>& args) {
    auto it = executables.find(name);
    if (it == executables.end()) {
        std::cout << "Executable not found: " << name << std::endl;
        return -1;
    }
    try {
        if (args.empty()) {
            boost::process::child c(it->second);
            c.wait();
            std::cout << name << " finished with code: " << c.exit_code() << std::endl;
            return c.exit_code();
        } else {
            boost::process::child c(it->second, boost::process::args(args));
            c.wait();
            std::cout << name << " finished with code: " << c.exit_code() << std::endl;
            return c.exit_code();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error running " << name << ": " << e.what() << std::endl;
        return -1;
    }
}

void launch::run_executables(const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& arguments) {
    std::vector<std::thread> threads;
    for (size_t i = 0; i < names.size(); ++i) {
        std::vector<std::string> args;
        if (i < arguments.size()) {
            args = arguments[i];
        }
        threads.emplace_back([this, name = names[i], args]() {
            run_executable(name, args);
        });
    }
    for (auto& t : threads) t.join();
}

int main(int argc, char** argv) {
    std::string yaml_path;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--yaml-path" && i + 1 < argc) {
            yaml_path = argv[i + 1];
            break;
        }
    }
    if (yaml_path.empty()) {
        std::cerr << "You must specify --yaml-path <path>" << std::endl;
        return 1;
    }
    NodesYamlParser parser(yaml_path);

    launch launch_instance(argc, argv, parser);
    // Si quieres seguir usando launch, puedes hacerlo aquí:
    // launch launch_instance(argc, argv, std::vector<std::string>{"executable_test", "executable_test2"});
    return 0;
}


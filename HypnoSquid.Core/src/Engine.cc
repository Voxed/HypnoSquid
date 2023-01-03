#include "HypnoSquid.Core/Engine.hh"

namespace hs {
namespace core {

std::thread Engine::invoke_system(const System &system) {
  return std::thread([&]() {
    std::unique_lock lk(system_resource_mutex);
    system_resource_cv.wait(lk, [&] {
      bool result = false;
      [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        result = (get<Is>(extensions).can_queue(*system.first) && ...);
      }
      (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});
      return result;
    });

    [&]<std::size_t... Is>(std::index_sequence<Is...>) { (get<Is>(extensions).queue(*system.first), ...); }
    (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});

    InvocationID inv = ++invocation_id;
    lk.unlock();
    system.first->invocation_id = inv;
    system.second();
    system.first->last_invocation_id = inv;
    lk.lock();

    [&]<std::size_t... Is>(std::index_sequence<Is...>) { (get<Is>(extensions).finished(*system.first), ...); }
    (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});

    lk.unlock();
    system_resource_cv.notify_all();
  });
}

void Engine::invoke_system_sync(const System &system) {
  invocation_id += 1;
  system.first->invocation_id = invocation_id;
  system.second();
  system.first->last_invocation_id = invocation_id;
}

void Engine::invoke_systems() {
  if (invocation_id == 0) {
    std::for_each(startup_systems.begin(), startup_systems.end(), [&](auto &system) { invoke_system_sync(system); });
  } else {
    std::vector<std::thread> threads(systems.size());
    std::transform(systems.begin(), systems.end(), threads.begin(),
                   [&](auto &system) { return invoke_system(system); });
    system_resource_cv.notify_all(); // Notify all threads to start the battle!
    std::for_each(threads.begin(), threads.end(), [&](auto &system) { system.join(); });
    std::for_each(synchronised_systems.begin(), synchronised_systems.end(),
                  [&](auto &system) { invoke_system_sync(system); });
  }
}

void Engine::load_plugin(const std::string &plugin_path) {
  std::cout << "Loading plugin: " << plugin_path << std::endl;
  void *plugin_handle = dlopen(plugin_path.c_str(), RTLD_LAZY);
  if (plugin_handle == nullptr) {
    std::cout << "Failed to load plugin " << plugin_path << ", file not found." << std::endl;
    exit(EXIT_FAILURE);
  }
  void (*init_plugin)(Engine &);
  init_plugin = (decltype(init_plugin))dlsym(plugin_handle, "init_plugin");
  if (init_plugin == nullptr) {
    std::cout << "Failed to load plugin " << plugin_path << ", no init function." << std::endl;
    exit(EXIT_FAILURE);
  }
  init_plugin(*this);
}

std::unique_ptr<SystemState> Engine::create_system_state() {
  return std::make_unique<SystemState>(
      SystemState{.system_id = next_system_id++, .last_invocation_id = 0, .invocation_id = 0});
}

Engine::Engine(const EngineCreateInfo &createInfo) {
  if (createInfo.use_config) {
    if (std::filesystem::exists(createInfo.config_path)) {
      std::cout << "Found engine configuration!" << std::endl;
      std::ifstream engine_config_file(createInfo.config_path);
      nlohmann::json engine_config = nlohmann::json::parse(engine_config_file);
      if (engine_config.contains("plugins")) {
        auto plugin_paths = engine_config["plugins"].get<std::vector<std::string>>();
        for (const auto &path : plugin_paths) {
          load_plugin(path);
        }
      }
    }
  }
  // Load explicit plugins
  for (const auto &path : createInfo.plugins) {
    load_plugin(path);
  }
}

void Engine::run() {
  while (running) {
    invoke_systems();
    [&]<std::size_t... Is>(std::index_sequence<Is...>) { (get<Is>(extensions).invoke_extension(invocation_id++), ...); }
    (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});
  }
}

} // namespace core
} // namespace hs
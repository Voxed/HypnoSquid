/**
 * @file Engine.hh
 * The file defining the main engine class.
 */

#pragma once

#include "InvocationID.hh"
#include "components/ComponentExtension.hh"

#include <condition_variable>
#include <dlfcn.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <tuple>
#include <vector>

namespace hs {
namespace core {

struct EngineCreateInfo {
  bool use_config = true;
  std::string config_path = "hypnosquid.json";
  std::vector<std::string> plugins;
};

class Engine {
  /*
   * Type definitions.
   */
  using SystemFunction = std::function<void()>;
  using System = std::pair<std::unique_ptr<SystemState>, SystemFunction>;

  /*
   * Engine extensions, these are used to decouple the engine from specific parameter instantiations.
   */
  std::tuple<ComponentExtension> extensions;

  /*
   * The different types of systems.
   */
  std::vector<System> systems;
  std::vector<System> synchronised_systems;
  std::vector<System> startup_systems;

  /*
   * Mutex and condition value used during system resource reservation.
   */
  std::mutex system_resource_mutex;
  std::condition_variable system_resource_cv;

  /*
   * Invocation id tracker.
   */
  InvocationID invocation_id = 0;

  /*
   * System id tracker.
   */
  SystemID next_system_id = 0;

  /*
   * Main loop condition.
   */
  bool running = true;

  /*
   * Pre-processors for system parameters.
   */
  template <class T> struct parameter_type : std::type_identity<T> {};
  template <class Q>
    requires(std::is_const_v<std::remove_reference_t<Q>> && std::is_reference_v<Q>)
  struct parameter_type<Q> : std::type_identity<std::remove_const_t<std::remove_reference_t<Q>>> {};
  template <class T> using parameter_type_t = typename parameter_type<T>::type;

  /**
   * Instantiate a system parameter.
   *
   * This is done once at the creation of a system, then the result will be bounded to the system function.
   *
   * @tparam P The type of the system parameter to instantiate.
   * @param t The type identity of the system parameter to instantiate.
   * @param state The state of the system which this parameter is being bound to.
   * @return The parameter value to be bound.
   */
  template <class P> P instantiate_parameter(std::type_identity<P> t, SystemState &state) {
    /*
     * In order to allow for references, they have to be handled a bit differently. In actuality, pointers might be more
     * appropriate. This is just to allow for more intuitive usage.
     */
    if constexpr (std::is_reference_v<P>) {
      std::optional<std::remove_reference_t<P> *> param = {};
      [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        (
            [&]() {
              if constexpr (requires(decltype(get<Is>(extensions)) &ext) { ext.instantiate_parameter(t, state); })
                if (!param.has_value())
                  param = &(get<Is>(extensions).instantiate_parameter(t, state));
            }(),
            ...);
      }
      (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});
      return *(param.value());
    } else {
      std::optional<P> param = {};
      [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        (
            [&]() {
              if constexpr (requires(decltype(get<Is>(extensions)) &ext) { ext.instantiate_parameter(t, state); })
                if (!param.has_value())
                  param.emplace(get<Is>(extensions).instantiate_parameter(t, state));
            }(),
            ...);
      }
      (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});
      return param.value();
    }
  }

  /**
   * Bind and instantiate the parameters to a system.
   *
   * @tparam Parameters The types of the parameters to be instantiated.
   * @param system The function to bind the parameters to.
   * @param system_state The state of the system which is being bound to.
   * @return A bound system function which can be invoked, now without any arguments.
   */
  template <class... Parameters>
  SystemFunction bind_system(std::function<void(Parameters...)> system, SystemState &system_state) {
    SystemRequirements requirements;
    auto parameters = std::tuple<parameter_type_t<Parameters>...>(
        instantiate_parameter(std::type_identity<parameter_type_t<Parameters>>(), system_state)...);
    SystemFunction func = [&, system, parameters]() {
      [&]<std::size_t... Is>(std::index_sequence<Is...>) { system(get<Is>(parameters)...); }
      (std::make_index_sequence<sizeof...(Parameters)>{});
    };
    return func;
  }

  /**
   * Invoke a system asynchronously.
   *
   * The system will attempt to reserve it's necessary resources and invoke itself every time another system finishes
   * an invocation.
   *
   * @param system The system to invoke.
   * @return The thread in which this system runs.
   */
  std::thread invoke_system(const System &system) {
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

  /**
   * Invokes a system synchronously.
   * @param system The system to invoke.
   */
  void invoke_system_sync(const System &system) {
    invocation_id += 1;
    system.first->invocation_id = invocation_id;
    system.second();
    system.first->last_invocation_id = invocation_id;
  }

  /**
   * Invoke all systems.
   *
   * This system will only run startup systems upon the first invocation. Thereafter it will block until synchronous and
   * asynchronous systems have been able to finish execution.
   */
  void invoke_systems() {
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

  /**
   * Load a plugin from a dynamic library.
   * @param plugin_path The path to the plugin dll.
   */
  void load_plugin(const std::string &plugin_path) {
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

  /**
   * Allocate a new system state in the system pool.
   * @return The newly allocated system state.
   */
  std::unique_ptr<SystemState> create_system_state() {
    return std::make_unique<SystemState>(
        SystemState{.system_id = next_system_id++, .last_invocation_id = 0, .invocation_id = 0});
  }

  /**
   * Create a new system from a function.
   * @tparam Parameters The parameters of the system.
   * @param system The function to create the system from.
   * @return The new system.
   */
  template <class... Parameters> System create_system(std::function<void(Parameters...)> system) {
    std::unique_ptr<SystemState> state = create_system_state();
    return System(std::move(state),
                  bind_system(std::function<void(Parameters...)>([&, system](Parameters... params) -> void {
                                system(std::forward<Parameters>(params)...);
                              }),
                              *state));
  }

public:
  /**
   * Construct the engine.
   * @param createInfo Optional creation hints.
   */
  explicit Engine(const EngineCreateInfo &createInfo = EngineCreateInfo()) {
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

  /**
   * Add an asynchronous system to the engine.
   * @tparam Parameters The parameters of the system.
   * @param system The function of the system.
   * @return A reference to the engine, to allow for method chaining.
   */
  template <class... Parameters> Engine &add_system(std::function<void(Parameters...)> system) {
    systems.push_back(create_system(system));
    return *this;
  }

  /**
   * Add an asynchronous system to the engine.
   * @tparam Parameters The parameters of the system.
   * @param system The function of the system.
   * @return A reference to the engine, to allow for method chaining.
   */
  template <class... Parameters> Engine &add_system(void (*system)(Parameters...)) {
    add_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

  /**
   * Add an synchronous system to the engine.
   * @tparam Parameters The parameters of the system.
   * @param system The function of the system.
   * @return A reference to the engine, to allow for method chaining.
   */
  template <class... Parameters> Engine &add_synchronised_system(std::function<void(Parameters...)> system) {
    synchronised_systems.push_back(create_system(system));
    return *this;
  }

  /**
   * Add an synchronous system to the engine.
   * @tparam Parameters The parameters of the system.
   * @param system The function of the system.
   * @return A reference to the engine, to allow for method chaining.
   */
  template <class... Parameters> Engine &add_synchronised_system(void (*system)(Parameters...)) {
    add_synchronised_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

  /**
   * Add an startup system to the engine.
   * @tparam Parameters The parameters of the system.
   * @param system The function of the system.
   * @return A reference to the engine, to allow for method chaining.
   */
  template <class... Parameters> Engine &add_startup_system(std::function<void(Parameters...)> system) {
    startup_systems.push_back(create_system(system));
    return *this;
  }

  /**
   * Add an startup system to the engine.
   * @tparam Parameters The parameters of the system.
   * @param system The function of the system.
   * @return A reference to the engine, to allow for method chaining.
   */
  template <class... Parameters> Engine &add_startup_system(void (*system)(Parameters...)) {
    add_startup_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

  /**
   * Start the engine main loop, will hang until the engine exits somehow.
   */
  void run() {
    while (running) {
      invoke_systems();
      [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        (get<Is>(extensions).invoke_extension(invocation_id++), ...);
      }
      (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});
    }
  }
};

} // namespace core
} // namespace hs
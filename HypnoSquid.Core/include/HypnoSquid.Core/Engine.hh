/**
 * This file contains a prototype for the engine.
 * Definitely going to need to decompose it and learn some better techniques.
 */

#pragma once

#include "Commands.hh"
#include "ComponentRegistry.hh"
#include "ComponentState.hh"
#include "ComponentStore.hh"
#include "Entity.hh"
#include "Filter.hh"
#include "Query.hh"
#include "World.hh"

#include <condition_variable>
#include <dlfcn.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <tuple>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hs {
namespace core {

struct EngineCreateInfo {
  bool use_config = true;
  std::string config_path = "hypnosquid.json";
  std::vector<std::string> plugins;
};

struct SystemRequirements {
  std::unordered_set<ComponentID> const_components;
  std::unordered_set<ComponentID> mutable_components;
};

using InvocationID =
    u_int64_t; // Needs to never run out! At 240 fps running 100000 systems, I'm counting 24372 years. Probably enough.

class ComponentExtension {
  std::unordered_map<ComponentID, u_int32_t> const_component_reference_count;
  std::unordered_set<ComponentID> mutable_component_references;
  std::vector<std::unique_ptr<QueryBuffer>> query_buffers;
  std::vector<std::unique_ptr<CommandBuffer>> command_queue;
  std::unordered_map<SystemID, SystemRequirements> requirements;
  EntityFactory entity_factory;
  ComponentRegistry component_registry;
  World world;

public:
  EntityFactory &instantiate_parameter(std::type_identity<EntityFactory &>, SystemState &state) {
    return entity_factory;
  }

  CommandBuffer &create_command_buffer() {
    command_queue.emplace_back(std::make_unique<CommandBuffer>());
    return *command_queue.back().get();
  }

  Commands instantiate_parameter(std::type_identity<Commands>, SystemState &state) {
    return Commands(create_command_buffer(), component_registry);
  }

  template <class... Args> QueryBuffer &retrieve_suitable_buffer() {
    for (auto &buff : query_buffers) {
      if (buff->template is_suitable<Args...>(component_registry)) {
        return *buff;
      }
    }
    query_buffers.emplace_back(std::unique_ptr<QueryBuffer>(
        new QueryBuffer(QueryBuffer::from_query(std::type_identity<Query<Args...>>(), component_registry))));
    return *(query_buffers.back().get());
  };

  template <class Res> void calculate_system_requirement(SystemRequirements &req) {
    if constexpr (!std::is_same_v<Entity, Res>) {
      if constexpr (std::is_const_v<Res>) {
        req.const_components.insert(component_registry.template get_component_id<Res>());
      } else {
        req.mutable_components.insert(component_registry.template get_component_id<Res>());
      }
    }
  }

  template <class First, class... Args>
  Query<First, Args...> instantiate_parameter(std::type_identity<Query<First, Args...>>, SystemState &state) {
    SystemID system_id = state.system_id;
    if constexpr (concepts::Filter<First>) {
      (calculate_system_requirement<Args>(requirements[system_id]), ...);
      return Query<First, Args...>(retrieve_suitable_buffer<Args...>(), state, component_registry, world);
    } else {
      calculate_system_requirement<First>(requirements[system_id]);
      (calculate_system_requirement<Args>(requirements[system_id]), ...);
      return Query<First, Args...>(retrieve_suitable_buffer<First, Args...>(), state, component_registry, world);
    }
  }

  void update_queries(Entity entity, InvocationID id) {
    for (auto &buff : query_buffers) {
      buff->update(entity, world, id);
    }
  }

  void add_component(Entity entity, ComponentID component_type,
                     std::unique_ptr<void, void (*)(void const *)> &component_data, InvocationID id) {
    AbstractComponentStore &store = world.get_component_store(component_type);
    if (!store.has_component(entity)) {
      store.add_component(entity, std::move(component_data), id);
      update_queries(entity, id);
    }
  }

  void remove_component(Entity entity, ComponentID component_type, InvocationID id) {
    AbstractComponentStore &store = world.get_component_store(component_type);
    if (store.has_component(entity)) {
      store.remove_component(entity);
      update_queries(entity, id);
    }
  }

  void invoke_extension(InvocationID id) {
    for (auto &command_buffer : command_queue) {
      for (auto &command : command_buffer->commands) {
        switch (command.type) {
        case ADD: {
          auto &action = command.action.add;
          auto ptr = std::unique_ptr<void, void (*)(void const *)>(action.component_data, action.component_destructor);
          add_component(action.entity_id, action.component_type, ptr, id);
          break;
        }
        case REMOVE: {
          auto &action = command.action.remove;
          remove_component(action.entity_id, action.component_type, id);
          break;
        }
        case EXIT:
          break;
        }
      }
      command_buffer->commands.clear();
    }
  }

  bool can_queue(SystemID system_id) {
    if (!requirements.contains(system_id))
      return true;
    auto &req = requirements[system_id];
    return std::ranges::all_of(req.mutable_components.begin(), req.mutable_components.end(),
                               [&](auto &c) {
                                 return !mutable_component_references.contains(c) &&
                                        const_component_reference_count[c] == 0;
                               }) &&
           std::ranges::all_of(req.const_components.begin(), req.const_components.end(),
                               [&](auto &c) { return !mutable_component_references.contains(c); });
  }

  void queue(SystemID system_id) {
    auto &req = requirements[system_id];
    mutable_component_references.insert(req.mutable_components.begin(), req.mutable_components.end());
    for (auto &m : req.const_components)
      const_component_reference_count[m]++;
  }

  void finished(SystemID system_id) {
    auto &req = requirements[system_id];
    for (auto &m : req.mutable_components)
      mutable_component_references.erase(m);
    for (auto &m : req.const_components)
      const_component_reference_count[m]--;
  }

  ComponentExtension() : world(component_registry) {}
};

class Engine {
  using System = std::pair<SystemID, std::function<void(InvocationID)>>;

  std::mutex system_resource_mutex;
  std::condition_variable system_resource_cv;

  std::vector<System> systems;
  std::vector<System> synchronised_systems;
  std::vector<System> startup_systems;
  std::vector<std::unique_ptr<SystemState>> system_states;

  InvocationID invocation_id = 0;
  SystemID next_system_id = 0;

  std::tuple<ComponentExtension> extensions;

  bool running = true;

  template <class T> struct parameter_type : std::type_identity<T> {};
  template <class Q>
    requires(concepts::specialization_of<Query, std::remove_const_t<std::remove_reference_t<Q>>>)
  struct parameter_type<Q> : std::type_identity<std::remove_const_t<std::remove_reference_t<Q>>> {};
  template <class T> using parameter_type_t = typename parameter_type<T>::type;

  template <class P>
  P instantiate_parameter(std::type_identity<P> t, SystemState &state) {
    if constexpr(std::is_reference_v<P>) {
      std::optional<std::remove_reference_t<P>*> param = {};
      [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        (
            [&]() {
              if (!param.has_value()) {
                std::cout << "HERE" << typeid(decltype(get<Is>(extensions))).name() << std::endl;
                if constexpr (requires(decltype(get<Is>(extensions)) &ext) {
                                ext.instantiate_parameter(t, state);
                              }) {
                  param = &(get<Is>(extensions).instantiate_parameter(t, state));
                  std::cout << typeid(P).name() << "DEFINED" << std::endl;
                }
              }
            }(),
            ...);
      }
      (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});
      return *(param.value());
    }else {
      std::optional<P> param = {};
      [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        (
            [&]() {
              if (!param.has_value()) {
                if constexpr (requires(decltype(get<Is>(extensions)) &ext) {
                                ext.instantiate_parameter(t, state);
                              }) {
                  param.emplace(get<Is>(extensions).instantiate_parameter(t, state));
                }
              }
            }(),
            ...);
      }
      (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});
      return param.value();
    }
  }

  template <class... Parameters>
  std::function<void(InvocationID)> bind_system(std::function<void(Parameters...)> system, SystemState &system_state) {
    SystemRequirements requirements;
    auto parameters = std::tuple<parameter_type_t<Parameters>...>(
        instantiate_parameter(std::type_identity<parameter_type_t<Parameters>>(), system_state)...);
    std::function<void(InvocationID)> func = [&, system, parameters](InvocationID _invocation_id) {
      system_state.invocation_id = _invocation_id;
      [&]<std::size_t... Is>(std::index_sequence<Is...>) { system(get<Is>(parameters)...); }
      (std::make_index_sequence<sizeof...(Parameters)>{});
      system_state.last_invocation_id = _invocation_id;
    };
    return func;
  }

  std::thread invoke_system(const System &system, u_int32_t index, SystemID system_id) {
    return std::thread([&, index]() {
      std::unique_lock lk(system_resource_mutex);
      system_resource_cv.wait(lk, [&] {
        bool result = false;
        [&]<std::size_t... Is>(std::index_sequence<Is...>) {
          result = (get<Is>(extensions).can_queue(system_id) && ...);
        }
        (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});
        return result;
      });

      [&]<std::size_t... Is>(std::index_sequence<Is...>) { (get<Is>(extensions).queue(system_id), ...); }
      (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});

      InvocationID inv = ++invocation_id;
      lk.unlock();
      system.second(inv);
      lk.lock();

      [&]<std::size_t... Is>(std::index_sequence<Is...>) { (get<Is>(extensions).finished(system_id), ...); }
      (std::make_index_sequence<std::tuple_size_v<decltype(extensions)>>{});

      lk.unlock();
      system_resource_cv.notify_all();
    });
  }

  void invoke_system_sync(const System &system) {
    invocation_id += 1;
    system.second(invocation_id);
  }

  void invoke_systems() {
    if (invocation_id == 0) {
      for (auto &system : startup_systems) {
        invoke_system_sync(system);
      }
    } else {
      std::vector<std::thread> threads;
      u_int32_t i = 0;
      for (auto &system : systems) {
        threads.push_back(invoke_system(system, i++, system.first));
      }
      system_resource_cv.notify_all(); // Notify all threads to start the battle!
      for (auto &t : threads)
        t.join();
      for (auto &system : synchronised_systems) {
        invoke_system_sync(system);
      }
    }
  }

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

  SystemState &create_system_state() {
    system_states.push_back(std::make_unique<SystemState>(
        SystemState{.system_id = next_system_id++, .last_invocation_id = 0, .invocation_id = 0}));
    return *system_states.back().get();
  }

  template <class... Parameters> System create_system(std::function<void(Parameters...)> system) {
    SystemState &state = create_system_state();
    return std::make_pair(state.system_id,
                          bind_system(std::function<void(Parameters...)>([&, system](Parameters... params) -> void {
                                        system(std::forward<Parameters>(params)...);
                                      }),
                                      state));
  }

public:
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

  template <class... Parameters> Engine &add_system(std::function<void(Parameters...)> system) {
    systems.push_back(create_system(system));
    return *this;
  }

  template <class... Parameters> Engine &add_system(void (*system)(Parameters...)) {
    add_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

  template <class... Parameters> Engine &add_synchronised_system(std::function<void(Parameters...)> system) {
    synchronised_systems.push_back(create_system(system));
    return *this;
  }

  template <class... Parameters> Engine &add_synchronised_system(void (*system)(Parameters...)) {
    add_synchronised_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

  template <class... Parameters> Engine &add_startup_system(std::function<void(Parameters...)> system) {
    startup_systems.push_back(create_system(system));
    return *this;
  }

  template <class... Parameters> Engine &add_startup_system(void (*system)(Parameters...)) {
    add_startup_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

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
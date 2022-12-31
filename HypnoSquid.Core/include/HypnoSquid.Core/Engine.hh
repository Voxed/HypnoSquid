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
#include "config.hh"

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

struct SystemRequirements {
  std::unordered_set<ComponentID> const_components;
  std::unordered_set<ComponentID> mutable_components;
};

using InvocationID =
    u_int64_t; // Needs to never run out! At 240 fps running 100000 systems, I'm counting 24372 years. Probably enough.

class Engine {
  using System = std::pair<SystemRequirements, std::function<void(InvocationID)>>;

  std::mutex system_resource_mutex;
  std::condition_variable system_resource_cv;
  std::unordered_map<ComponentID, u_int32_t> const_component_reference_count;
  std::unordered_set<ComponentID> mutable_component_references;

  std::vector<std::unique_ptr<QueryBuffer>> query_buffers;
  std::vector<System> systems;
  std::vector<System> synchronised_systems;
  std::vector<System> startup_systems;
  std::vector<std::unique_ptr<SystemState>> system_states;
  std::vector<std::unique_ptr<CommandBuffer>> command_queue;

  InvocationID invocation_id = 0;

  EntityFactory entity_factory;

  ComponentRegistry component_registry;
  ComponentStore store;

  bool running = true;

  template <concepts::Filter... Filters>
  std::unordered_set<Entity> apply_all_filters(std::type_identity<filters::All<Filters...>>,
                                               std::unordered_set<Entity> entities, SystemState system_state) {
    std::unordered_set<Entity> filtered = std::move(entities);
    (
        [&]() {
          std::unordered_set<Entity> old_filtered = std::move(filtered);
          filtered.clear();
          for (const auto &e : apply_filter<Filters>(old_filtered, system_state))
            filtered.insert(e);
        }(),
        ...);
    return filtered;
  }

  template <concepts::Filter... Filters>
  std::unordered_set<Entity> apply_any_filters(std::type_identity<filters::All<Filters...>>,
                                               std::unordered_set<Entity> entities, SystemState system_state) {
    std::unordered_set<Entity> filtered;
    (
        [&]() {
          for (const auto &e : apply_filter<Filters>(entities, system_state))
            filtered.insert(e);
        }(),
        ...);
    return filtered;
  }

  template <concepts::Nop Filter>
  std::unordered_set<Entity> apply_filter(std::unordered_set<Entity> entities, SystemState system_state) {
    return entities;
  }

  template <concepts::Changed Filter>
  std::unordered_set<Entity> apply_filter(const std::unordered_set<Entity> &entities, SystemState system_state) {
    std::unordered_set<Entity> filtered;
    for (const Entity &e : entities)
      if (store.get_component_state<typename Filter::component_type>(e)->has_changed(system_state))
        filtered.insert(e);
    return filtered;
  }

  template <concepts::Not Filter>
  std::unordered_set<Entity> apply_filter(const std::unordered_set<Entity> &entities, SystemState system_state) {
    std::unordered_set<Entity> filtered;
    for (const Entity &e : entities)
      if (!store.has_component<typename Filter::component_type>(e))
        filtered.insert(e);
    return filtered;
  }

  template <concepts::All Filter>
  std::unordered_set<Entity> apply_filter(const std::unordered_set<Entity> &entities, SystemState system_state) {
    return apply_all_filters(std::type_identity<Filter>(), entities, system_state);
  }

  template <concepts::Any Filter>
  std::unordered_set<Entity> apply_filter(const std::unordered_set<Entity> &entities, SystemState system_state) {
    return apply_any_filters(std::type_identity<Filter>(), entities, system_state);
  }

  EntityFactory &instantiate_parameter(std::type_identity<EntityFactory &>, SystemState &state,
                                       SystemRequirements &requirements) {
    return entity_factory;
  }

  CommandBuffer &create_command_buffer() {
    command_queue.emplace_back(std::make_unique<CommandBuffer>());
    return *command_queue.back().get();
  }

  Commands instantiate_parameter(std::type_identity<Commands>, SystemState &state, SystemRequirements &requirements) {
    return Commands(create_command_buffer(), component_registry);
  }

  template <class... Args> QueryBuffer &retrieve_suitable_buffer() {
    for (auto &buff : query_buffers) {
      if (buff->template is_suitable<Args...>(component_registry)) {
        std::cout << "FOUND SUITABLE" << std::endl;
        return *(buff.get());
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
  Query<First, Args...> instantiate_parameter(std::type_identity<Query<First, Args...>>, SystemState &state,
                                              SystemRequirements &requirements) {
    if constexpr (concepts::Filter<First>) {
      (calculate_system_requirement<Args>(requirements), ...);
      return Query<First, Args...>(retrieve_suitable_buffer<Args...>(), state, component_registry, store);
    } else {
      calculate_system_requirement<First>(requirements);
      (calculate_system_requirement<Args>(requirements), ...);
      return Query<First, Args...>(retrieve_suitable_buffer<First, Args...>(), state, component_registry, store);
    }
  }

  template <class... Parameters>
  System bind_system(std::function<void(Parameters...)> system, SystemState &system_state) {
    SystemRequirements requirements;
    auto parameters = std::tuple<Parameters...>(
        instantiate_parameter(std::type_identity<Parameters>(), system_state, requirements)...);
    std::function<void(InvocationID)> func = [&, system, parameters](InvocationID _invocation_id) {
      system_state.invocation_id = _invocation_id;
      [&]<std::size_t... Is>(std::index_sequence<Is...>) { system(get<Is>(parameters)...); }
      (std::make_index_sequence<sizeof...(Parameters)>{});
      system_state.last_invocation_id = _invocation_id;
    };
    return std::make_pair(requirements, func);
  }

  void update_queries(Entity entity) {
    for (auto &buff : query_buffers) {
      bool belongs = true;
      for (auto &l : buff->layout)
        if (l.type == COMPONENT && !store.has_component(l.data.component_type, entity)) {
          belongs = false;
          break;
        }
      if (!buff->items.contains(entity) && belongs) {
        std::vector<QueryBufferItem> items;
        for (auto &l : buff->layout)
          switch (l.type) {
          case COMPONENT: {
            items.push_back(QueryBufferItem{.component = {store.get_component_data(l.data.component_type, entity),
                                                          *store.get_component_state(l.data.component_type, entity)}});
            break;
          }
          }
        buff->items.emplace(entity, items);
        buff->last_changed = invocation_id;
      }
      if (buff->items.contains(entity) && !belongs) {
        buff->items.erase(entity);
        buff->last_changed = invocation_id;
      }
    }
  }

  void add_component(Entity entity, ComponentID component_type,
                     std::unique_ptr<void, void (*)(void const *)> &component_data) {
    if (!store.has_component(component_type, entity)) {
      store.add_component(component_type, entity, std::move(component_data), invocation_id);
      update_queries(entity);
    }
  }

  void remove_component(Entity entity, ComponentID component_type) {
    if (store.has_component(component_type, entity)) {
      store.remove_component(component_type, entity);
      update_queries(entity);
    }
  }

  void process_commands() {
    for (auto &command_buffer : command_queue) {
      for (auto &command : command_buffer->commands) {
        switch (command.type) {
        case ADD: {
          auto &action = command.action.add;
          auto ptr = std::unique_ptr<void, void (*)(void const *)>(action.component_data, action.component_destructor);
          add_component(action.entity_id, action.component_type, ptr);
          break;
        }
        case REMOVE: {
          auto &action = command.action.remove;
          remove_component(action.entity_id, action.component_type);
          break;
        }
        case EXIT:
          running = false;
          break;
        }
      }
      command_buffer->commands.clear();
    }
  }

  std::thread invoke_system(const System &system, u_int32_t index) {
    return std::thread([&, index]() {
      std::unique_lock lk(system_resource_mutex);

      system_resource_cv.wait(lk, [&] {
        // Calculate whether the system requirements can be met at this time.
        // Mutable requirements
        for (auto &m : system.first.mutable_components) {
          if (mutable_component_references.contains(m) || const_component_reference_count[m] > 0) {
            return false;
          }
        }
        // Constant requirements
        for (auto &m : system.first.const_components) {
          if (mutable_component_references.contains(m)) {
            return false;
          }
        }

        return true;
      });

      // We're good! Acquire resources!
      for (auto &m : system.first.mutable_components) {
        mutable_component_references.insert(m);
      }
      for (auto &m : system.first.const_components) {
        if (!system.first.mutable_components.contains(
                m)) { // If we already have mutable ownership, we don't need constant ownership.
          const_component_reference_count[m]++;
        }
      }

      invocation_id++;
      InvocationID inv = invocation_id;
      lk.unlock();

      invocation_id++;
      system.second(inv);

      lk.lock();

      // Release resources
      for (auto &m : system.first.mutable_components) {
        mutable_component_references.erase(m);
      }
      for (auto &m : system.first.const_components) {
        if (!system.first.mutable_components.contains(m)) {
          const_component_reference_count[m]--;
        }
      }

      lk.unlock();

      // Notify that new resources might be available!
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
        threads.push_back(invoke_system(system, i++));
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
      exit(-1);
    }
    void (*init_plugin)(Engine &);
    init_plugin = (decltype(init_plugin))dlsym(plugin_handle, "init_plugin");
    if (init_plugin == nullptr) {
      std::cout << "Failed to load plugin " << plugin_path << ", no init function." << std::endl;
      exit(-1);
    }
    init_plugin(*this);
  }

  SystemState &create_system_state() {
    system_states.push_back(std::make_unique<SystemState>(SystemState{.last_invocation_id = 0, .invocation_id = 0}));
    return *system_states.back().get();
  }

  template <class... Parameters> System create_system(std::function<void(Parameters...)> system) {
    SystemState &state = create_system_state();
    return bind_system(std::function<void(Parameters...)>(
                           [&, system](Parameters... params) -> void { system(std::forward<Parameters>(params)...); }),
                       state);
  }

public:
  Engine() : store(component_registry) {
    if (std::filesystem::exists(HS_CFG_PATH)) {
      std::cout << "Found engine configuration!" << std::endl;
      std::ifstream engine_config_file(HS_CFG_PATH);
      nlohmann::json engine_config = nlohmann::json::parse(engine_config_file);
      if (engine_config.contains("plugins")) {
        auto plugin_paths = engine_config["plugins"].get<std::vector<std::string>>();
        for (const auto &path : plugin_paths) {
          load_plugin(path);
        }
      }
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
    std::cout << "Query buffers: " << query_buffers.size() << std::endl;
    while (running) {
      invoke_systems();
      process_commands();
// #define PRINT_MEMORY
#ifdef PRINT_MEMORY
      size_t total_size_components = 0;
      for (auto &p : data) {
        total_size_components += p.second.size();
      }
      size_t total_size_entities = 0;
      for (auto &p : entity_components) {
        total_size_entities += p.second.size();
      }
      std::cout << data.size() << ", " << total_size_components << ", " << entity_components.size() << ", "
                << total_size_entities << std::endl;
#endif
    }
  }
};

} // namespace core
} // namespace hs
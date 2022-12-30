/**
 * This file contains a prototype for the engine.
 * Definitely going to need to decompose it and learn some better techniques.
 */

#pragma once

#include "Commands.hh"
#include "ComponentRegistry.hh"
#include "ComponentState.hh"
#include "Entity.hh"
#include "Filter.hh"
#include "Query.hh"

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

using SystemID = u_int32_t;
using ParameterID = u_int32_t;

class Engine {
  using System = std::pair<SystemState, std::function<void(SystemState &)>>;

  std::unordered_map<
      ComponentID, std::unordered_map<Entity, std::pair<ComponentState, std::unique_ptr<void, void (*)(void const *)>>>>
      data;

  std::unordered_map<Entity, std::unordered_set<ComponentID>> entity_components;

  SystemID next_system_id = 0;
  std::unordered_map<SystemID, std::unordered_map<ParameterID, std::unique_ptr<void, void (*)(void const *)>>> queries;
  std::vector<std::unique_ptr<QueryBuffer>> query_buffers;

  u_int64_t invocation_id = 0;
  std::vector<System> systems;
  std::vector<System> synchronised_systems;
  std::vector<System> startup_systems;
  std::vector<CommandBuffer> command_queue;
  EntityFactory entity_factory;

  ComponentRegistry component_registry;

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
      if (data.at(component_registry.template get_component_id<typename Filter::component_type>())
              .at(e)
              .first.has_changed(system_state))
        filtered.insert(e);
    return filtered;
  }

  template <concepts::Not Filter>
  std::unordered_set<Entity> apply_filter(const std::unordered_set<Entity> &entities, SystemState system_state) {
    std::unordered_set<Entity> filtered;
    for (const Entity &e : entities)
      if (!data.at(component_registry.template get_component_id<typename Filter::component_type>()).contains(e))
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

  EntityFactory &instantiate_parameter(std::type_identity<EntityFactory &>, SystemState &state, ParameterID, SystemID) {
    return entity_factory;
  }

  Commands instantiate_parameter(std::type_identity<Commands>, SystemState &state, ParameterID, SystemID) {
    CommandBuffer &command_buffer = command_queue.emplace_back();
    return Commands(command_buffer, component_registry);
  }

  template <class First, class... Args>
  Query<First, Args...> &instantiate_parameter(std::type_identity<Query<First, Args...> &>, SystemState &state,
                                               ParameterID parameter_id, SystemID system_id) {
    if (!queries[system_id].contains(parameter_id)) {
      query_buffers.emplace_back(std::unique_ptr<QueryBuffer>(
          new QueryBuffer(QueryBuffer::from_query(std::type_identity<Query<First, Args...>>(), component_registry))));
      if constexpr (concepts::Filter<First>) {
        queries[system_id].emplace(parameter_id, make_unique_void(new Query<First, Args...>(
                                                     *(query_buffers.back().get()), state, component_registry,
                                                     [this, &state](const std::unordered_set<Entity> &entities) {
                                                       return apply_filter<First>(entities, state);
                                                     })));
      } else {
        queries[system_id].emplace(parameter_id, make_unique_void(new Query<First, Args...>(
                                                     *(query_buffers.back().get()), state, component_registry)));
      }
    }
    return *static_cast<Query<First, Args...> *>(
        queries[system_id].at(parameter_id).get()); // create_query<Query<Args...>, Args...>(state);
  }

  template <class... Parameters>
  std::function<void(SystemState &)> bind_system(std::function<void(SystemState &, Parameters...)> system) {
    SystemID system_id = next_system_id++;
    std::function<void(SystemState &)> func = [&, system_id, system](SystemState &state) {
      int i = 0;
      system(state, instantiate_parameter(
                        std::type_identity<Parameters>(), state,
                        [&]() { return i++; }() /* Really stupid patch for sequenced modifications */, system_id)...);
    };
    return func;
  }

  void update_queries(Entity entity) {
    for (auto &buff : query_buffers) {
      bool belongs = true;
      for (auto &l : buff->layout) {
        if (l.type == COMPONENT) {
          if (!entity_components[entity].contains(l.data.component_type)) {
            belongs = false;
            break;
          }
        }
      }
      if (!buff->items.contains(entity) && belongs) {
        std::vector<QueryBufferItem> items;
        for (auto &l : buff->layout) {
          switch (l.type) {
          case COMPONENT: {
            auto &pair = data[l.data.component_type].at(entity);
            items.push_back(QueryBufferItem{.component = {pair.second.get(), pair.first}});
            break;
          }
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
    data[component_type].emplace(
        entity, std::move(std::make_pair(ComponentState{.last_changed = invocation_id}, std::move(component_data))));

    entity_components[entity].insert(component_type);

    update_queries(entity);
  }

  void remove_component(Entity entity, ComponentID component_type) {
    data[component_type].erase(entity);

    entity_components[entity].erase(component_type);

    update_queries(entity);
  }

  void process_commands() {
    for (auto &command_buffer : command_queue) {
      for (auto &command : command_buffer.commands) {
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
        }
      }
    }
    command_queue.clear();
  }

  void invoke_systems() {
    if (invocation_id == 0) {
      for (auto &system : startup_systems) {
        invocation_id++;
        system.first.invocation_id = invocation_id;
        system.second(system.first);
        system.first.last_invocation_id = invocation_id;
      }
    }
    for (auto &system : systems) {
      invocation_id++;
      system.first.invocation_id = invocation_id;
      system.second(system.first);
      system.first.last_invocation_id = invocation_id;
    }
    for (auto &system : synchronised_systems) {
      invocation_id++;
      system.first.invocation_id = invocation_id;
      system.second(system.first);
      system.first.last_invocation_id = invocation_id;
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

public:
  Engine() {
    if (std::filesystem::exists("hypnosquid.json")) {
      std::cout << "Found engine configuration!" << std::endl;
      std::ifstream engine_config_file("hypnosquid.json");
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
    systems.push_back(std::make_pair(SystemState{.last_invocation_id = 0, .invocation_id = 0},
                                     bind_system(std::function<void(SystemState &, Parameters...)>(
                                         [&, system](SystemState &, Parameters... params) -> void {
                                           system(std::forward<Parameters>(params)...);
                                         }))));
    return *this;
  }

  template <class... Parameters> Engine &add_system(void (*system)(Parameters...)) {
    add_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

  template <class... Parameters> Engine &add_synchronised_system(std::function<void(Parameters...)> system) {
    synchronised_systems.push_back(std::make_pair(SystemState{.last_invocation_id = 0, .invocation_id = 0},
                                                  bind_system(std::function<void(SystemState &, Parameters...)>(
                                                      [&, system](SystemState &, Parameters... params) -> void {
                                                        system(std::forward<Parameters>(params)...);
                                                      }))));
    return *this;
  }

  template <class... Parameters> Engine &add_synchronised_system(void (*system)(Parameters...)) {
    add_synchronised_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

  template <class... Parameters> Engine &add_startup_system(std::function<void(Parameters...)> system) {
    startup_systems.push_back(std::make_pair(SystemState{.last_invocation_id = 0, .invocation_id = 0},
                                             bind_system(std::function<void(SystemState &, Parameters...)>(
                                                 [&, system](SystemState &, Parameters... params) -> void {
                                                   system(std::forward<Parameters>(params)...);
                                                 }))));
    return *this;
  }

  template <class... Parameters> Engine &add_startup_system(void (*system)(Parameters...)) {
    add_startup_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

  void run() {
    while (true) {
      invoke_systems();
      process_commands();
    }
  }
};

} // namespace core
} // namespace hs
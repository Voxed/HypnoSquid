/**
 * This file contains a prototype for the engine.
 * Definitely going to need to decompose it and learn some better techniques.
 */

#pragma once

#include "Commands.hh"
#include "ComponentState.hh"
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

using Entity = u_int32_t;

// Atomic entity counter, to allow for immediate id allocation in concurrent
// command queues.
class EntityFactory {
  u_int32_t next_entity_id = 0;
  std::mutex mut;

public:
  u_int32_t create_entity() {
    mut.lock();
    u_int32_t id = next_entity_id++;
    mut.unlock();
    return id;
  }
};

class Engine {
  using System = std::pair<SystemState, std::function<void(SystemState)>>;

  std::unordered_map<
      std::type_index,
      std::unordered_map<
          u_int32_t, std::pair<ComponentState,
                               std::unique_ptr<void, void (*)(void const *)>>>>
      data;

  u_int64_t invocation_id = 0;
  std::vector<System> systems;
  std::vector<System> synchronised_systems;
  std::vector<System> startup_systems;
  std::vector<CommandBuffer> command_queue;
  EntityFactory entity_factory;

  template <class T>
  T query(u_int32_t entity, SystemState system_state) = delete;

  template <> u_int32_t query(u_int32_t entity, SystemState system_state) {
    return entity;
  }

  template <concepts::ComponentReference T>
  T query(u_int32_t entity, SystemState system_state) {
    using ComponentType = typename T::value_type;
    auto &component = data[typeid(ComponentType)].at(entity);
    concepts::ComponentReference auto value =
        T(static_cast<ComponentType *>(component.second.get()), component.first,
          system_state);
    return value;
  }

  template <class... Values>
  std::tuple<Values...> query_tuple(u_int32_t entity,
                                    SystemState system_state) {
    return std::make_tuple(query<Values>(entity, system_state)...);
  }

  template <class... Filters>
  std::unordered_set<Entity>
  apply_all_filters(std::type_identity<filters::All<Filters...>>,
                    std::unordered_set<Entity> entities,
                    SystemState system_state) {
    std::unordered_set<Entity> filtered = std::move(entities);
    (
        [&]() {
          std::unordered_set<Entity> old_filtered = std::move(filtered);
          filtered.clear();
          for (auto e : apply_filter<Filters>(old_filtered, system_state))
            filtered.insert(e);
        }(),
        ...);
    return filtered;
  }

  template <class... Filters>
  std::unordered_set<Entity>
  apply_any_filters(std::type_identity<filters::All<Filters...>>,
                    std::unordered_set<Entity> entities,
                    SystemState system_state) {
    std::unordered_set<Entity> filtered;
    (
        [&]() {
          for (auto e : apply_filter<Filters>(entities, system_state))
            filtered.insert(e);
        }(),
        ...);
    return filtered;
  }

  template <class Filter>
  std::unordered_set<Entity> apply_filter(std::unordered_set<Entity> entities,
                                          SystemState system_state) {

    std::unordered_set<Entity> filtered;
    if constexpr (concepts::Changed<Filter>) {
      for (Entity e : entities)
        if (data.at(typeid(typename Filter::component_type))
                .at(e)
                .first.has_changed(system_state))
          filtered.insert(e);
    } else if constexpr (concepts::Not<Filter>) {
      for (Entity e : entities)
        if (!data.at(typeid(typename Filter::component_type)).contains(e))
          filtered.insert(e);
    } else if constexpr (concepts::All<Filter>) {
      filtered = apply_all_filters(std::type_identity<Filter>(), entities,
                                   system_state);
    } else if constexpr (concepts::Any<Filter>) {
      filtered = apply_any_filters(std::type_identity<Filter>(), entities,
                                   system_state);
    } else if constexpr (concepts::Nop<Filter>) {
      filtered = entities;
    }

    return filtered;
  }

  std::unordered_set<u_int32_t>
  intersect_stores(std::vector<std::type_index> to_intersect) {
    std::unordered_set<u_int32_t> entities;
    if (to_intersect.empty()) {
      return entities;
    }
    for (auto &p : data[to_intersect.back()]) {
      entities.insert(p.first);
    }
    to_intersect.pop_back();
    std::unordered_set<u_int32_t> intersected;
    for (auto e : entities) {
      bool bad = false;
      for (auto &i : to_intersect) {
        if (!data[i].contains(e)) {
          bad = true;
          break;
        }
      }
      if (!bad) {
        intersected.insert(e);
      }
    }
    return intersected;
  }

  template <class Query, class Filter, class... Args>
  Query create_query(SystemState system_state) {
    std::vector<std::type_index> stores;
    (
        [&]() {
          if constexpr (!std::is_same_v<Args, Entity>) {
            stores.emplace_back(typeid(Args));
          }
        }(),
        ...);
    std::vector<std::tuple<create_component_reference_t<Args>...>> components;
    std::unordered_set<u_int32_t> entities = intersect_stores(stores);
    entities = apply_filter<Filter>(entities, system_state);
    for (u_int32_t e : entities) {
      components.push_back(
          query_tuple<create_component_reference_t<Args>...>(e, system_state));
    }
    return Query{.entities = std::move(components)};
  }

  template <class Query, class First, class... Args>
    requires(!concepts::Filter<First>)
  Query create_query(SystemState state) {
    return create_query<Query, filters::Nop, First, Args...>(state);
  }

  template <class T>
  T instantiate_parameter(std::type_identity<T>, SystemState state);

  template <>
  EntityFactory &instantiate_parameter(std::type_identity<EntityFactory &>,
                                       SystemState state) {
    return entity_factory;
  }

  template <>
  Commands instantiate_parameter(std::type_identity<Commands>,
                                 SystemState state) {
    CommandBuffer &command_buffer = command_queue.emplace_back();
    return Commands(command_buffer);
  }

  template <class... Args>
  Query<Args...> instantiate_parameter(std::type_identity<Query<Args...>>,
                                       SystemState state) {
    return create_query<Query<Args...>, Args...>(state);
  }

  template <class... Parameters>
  std::function<void(SystemState)>
  bind_system(std::function<void(SystemState, Parameters...)> system) {
    std::function<void(SystemState)> func = [=](SystemState state) {
      system(state,
             instantiate_parameter(std::type_identity<Parameters>(), state)...);
    };
    return func;
  }

  void
  add_component(u_int32_t entity, std::type_index component_type,
                std::unique_ptr<void, void (*)(void const *)> &component_data) {
    data[component_type].emplace(
        entity,
        std::move(std::make_pair(ComponentState{.last_changed = invocation_id},
                                 std::move(component_data))));
  }

  void remove_component(u_int32_t entity, std::type_index component_type) {
    data[component_type].erase(entity);
  }

  void process_commands() {
    for (auto &command_buffer : command_queue) {
      for (auto &command : command_buffer.commands) {
        switch (command.type) {
        case ADD: {
          auto &action = command.action.add;
          auto ptr = std::unique_ptr<void, void (*)(void const *)>(
              action.component_data, action.component_destructor);
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
      std::cout << "Failed to load plugin " << plugin_path
                << ", file not found." << std::endl;
      exit(-1);
    }
    void (*init_plugin)(Engine &);
    init_plugin = (decltype(init_plugin))dlsym(plugin_handle, "init_plugin");
    if (init_plugin == nullptr) {
      std::cout << "Failed to load plugin " << plugin_path
                << ", no init function." << std::endl;
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
        auto plugin_paths =
            engine_config["plugins"].get<std::vector<std::string>>();
        for (const auto &path : plugin_paths) {
          load_plugin(path);
        }
      }
    }
  }

  template <class... Parameters>
  Engine &add_system(std::function<void(Parameters...)> system) {
    systems.push_back(std::make_pair(
        SystemState(),
        bind_system(std::function<void(SystemState, Parameters...)>(
            [=](SystemState, Parameters... params) -> void {
              system(std::forward<Parameters>(params)...);
            }))));
    return *this;
  }

  template <class... Parameters>
  Engine &add_system(void (*system)(Parameters...)) {
    add_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

  template <class... Parameters>
  Engine &add_synchronised_system(std::function<void(Parameters...)> system) {
    synchronised_systems.push_back(std::make_pair(
        SystemState(),
        bind_system(std::function<void(SystemState, Parameters...)>(
            [=](SystemState, Parameters... params) -> void {
              system(std::forward<Parameters>(params)...);
            }))));
    return *this;
  }

  template <class... Parameters>
  Engine &add_synchronised_system(void (*system)(Parameters...)) {
    add_synchronised_system(
        std::function<std::remove_pointer_t<decltype(system)>>(system));
    return *this;
  }

  template <class... Parameters>
  Engine &add_startup_system(std::function<void(Parameters...)> system) {
    startup_systems.push_back(std::make_pair(
        SystemState(),
        bind_system(std::function<void(SystemState, Parameters...)>(
            [=](SystemState, Parameters... params) -> void {
              system(std::forward<Parameters>(params)...);
            }))));
    return *this;
  }

  template <class... Parameters>
  Engine &add_startup_system(void (*system)(Parameters...)) {
    add_startup_system(
        std::function<std::remove_pointer_t<decltype(system)>>(system));
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
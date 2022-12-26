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
  std::unordered_map<
      std::type_index,
      std::unordered_map<
          u_int32_t, std::pair<ComponentState,
                               std::unique_ptr<void, void (*)(void const *)>>>>
      data;

  u_int64_t invocation_id = 0;
  std::vector<std::function<void()>> systems;
  std::vector<CommandBuffer> command_queue;
  EntityFactory entity_factory;

  template <class Tuple, class Iterator, class... Values>
  Tuple query_tuple(u_int32_t entity, u_int64_t last_invocation_id,
                    Values... values) {
    if constexpr (!Iterator::empty) {
      using Type = typename Iterator::type;
      using Next = typename Iterator::next;
      if constexpr (std::is_same_v<u_int32_t, Type>) {
        auto value = entity;
        return query_tuple<Tuple, Next>(entity, last_invocation_id, values...,
                                        value);
      } else if constexpr (concepts::ComponentReference<Type>) {
        using ComponentType = typename Type::value_type;
        auto &component = data[typeid(ComponentType)].at(entity);
        auto value = Type(static_cast<ComponentType *>(component.second.get()),
                          component.first, invocation_id, last_invocation_id);
        return query_tuple<Tuple, Next>(entity, last_invocation_id, values...,
                                        value);
      }
    } else {
      return std::make_tuple(values...);
    }
  }

  template <class FilterIterator>
  std::unordered_set<Entity>
  apply_all_filters(std::unordered_set<Entity> entities,
                    u_int64_t last_invocation_id) {
    if constexpr (FilterIterator::empty) {
      return entities;
    } else {
      std::unordered_set<Entity> filtered;
      using Filter = typename FilterIterator::type;
      for (Entity e : entities) {
        if constexpr (concepts::Changed<Filter>) {
          if (data.at(typeid(typename Filter::component_type))
                  .at(e)
                  .first.has_changed(last_invocation_id))
            filtered.insert(e);
        } else if constexpr (concepts::Not<Filter>) {
          if (!data.at(typeid(typename Filter::component_type)).contains(e))
            filtered.insert(e);
        }
      }
      return apply_all_filters<typename FilterIterator::next>(
          filtered, last_invocation_id);
    }
  }

  template <class FilterIterator>
  std::unordered_set<Entity>
  apply_any_filters(std::unordered_set<Entity> filtered,
                    std::unordered_set<Entity> entities,
                    u_int64_t last_invocation_id) {
    if constexpr (FilterIterator::empty) {
      return filtered;
    } else {
      using Filter = typename FilterIterator::type;
      for (auto e : apply_filter<Filter>(entities, last_invocation_id))
        filtered.insert(e);
      return apply_any_filters<typename FilterIterator::next>(
          filtered, entities, last_invocation_id);
    }
  }

  template <class Filter>
  std::unordered_set<Entity> apply_filter(std::unordered_set<Entity> entities,
                                          u_int64_t last_invocation_id) {

    std::unordered_set<Entity> filtered;
    if constexpr (concepts::Changed<Filter>) {
      for (Entity e : entities)
        if (data.at(typeid(typename Filter::component_type))
                .at(e)
                .first.has_changed(last_invocation_id))
          filtered.insert(e);
    } else if constexpr (concepts::Not<Filter>) {
      for (Entity e : entities)
        if (!data.at(typeid(typename Filter::component_type)).contains(e))
          filtered.insert(e);
    } else if constexpr (concepts::All<Filter>) {
      filtered = apply_all_filters<typename Filter::iterator>(
          entities, last_invocation_id);
    } else if constexpr (concepts::Any<Filter>) {
      filtered = apply_any_filters<typename Filter::iterator>(
          {}, entities, last_invocation_id);
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

  template <class T> T create_query(u_int64_t last_invocation) {
    using Tuple = typename decltype(T::entities)::value_type;
    using Iterator = typename T::component_type_iterator;
    using Filter = typename T::filter;
    std::vector<std::type_index> stores =
        Iterator::template get_type_indices<u_int32_t>();
    std::vector<Tuple> components;
    std::unordered_set<u_int32_t> entities = intersect_stores(stores);
    entities = apply_filter<Filter>(entities, last_invocation);
    for (u_int32_t e : entities) {
      components.push_back(query_tuple<Tuple, Iterator>(e, last_invocation));
    }
    return T{.entities = std::move(components)};
  }

  template <class First, class... Parameters>
    requires concepts::is_in<First, Commands, EntityFactory &> ||
             concepts::QueryConcept<std::remove_reference_t<First>>
  std::function<void()>
  bind_system(std::function<void(First, Parameters...)> system) {
    std::function<void(Parameters...)> func = [this,
                                               system](Parameters... params) {
      if constexpr (std::is_same_v<Commands, First>) {
        CommandBuffer &command_buffer = command_queue.emplace_back();
        system(Commands(command_buffer), params...);
      } else if constexpr (std::is_same_v<EntityFactory &, First>) {
        system(entity_factory, params...);
      } else if constexpr (concepts::QueryConcept<
                               std::remove_reference_t<First>>) {
        static u_int64_t last_invocation_id = 0;
        std::remove_reference_t<First> query =
            create_query<std::remove_reference_t<First>>(last_invocation_id);
        last_invocation_id = invocation_id;
        system(query, params...);
      }
    };
    if constexpr (sizeof...(Parameters) == 0)
      return func;
    else
      return bind_system(func);
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
  }

  void invoke_systems() {
    for (auto &system : systems) {
      invocation_id++;
      system();
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
  void add_system(std::function<void(Parameters...)> system) {
    systems.push_back(bind_system(std::function<void(Parameters...)>(
        [system](Parameters... params) -> void {
          system(std::forward<Parameters>(params)...);
        })));
  }

  template <class... Parameters>
  void add_system(void (*system)(Parameters...)) {
    add_system(std::function<std::remove_pointer_t<decltype(system)>>(system));
  }

  void test() {
    invoke_systems();
    process_commands();
    invoke_systems();
    invoke_systems();
  }
};

} // namespace core
} // namespace hs
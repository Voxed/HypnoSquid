/**
 * This file contains a prototype for the engine.
 * Definitely going to need to decompose it and learn some better techniques.
 */

#pragma once

#include "Commands.hh"
#include "ComponentState.hh"
#include "Filter.hh"
#include "Query.hh"

#include <functional>
#include <iostream>
#include <memory>
#include <tuple>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
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
                    Values... vals) {
    if constexpr (!Iterator::empty) {
      if constexpr (std::is_same_v<u_int32_t, typename Iterator::type>) {
        auto value = entity;
        return query_tuple<Tuple, typename Iterator::next>(
            entity, last_invocation_id, vals..., value);
      } else if constexpr (requires {
                             Iterator::type::is_component_reference;
                           }) {
        auto &component =
            data[typeid(typename Iterator::type::value_type)].at(entity);
        auto value = typename Iterator::type(
            static_cast<typename Iterator::type::value_type *>(
                component.second.get()),
            component.first, invocation_id, last_invocation_id);
        return query_tuple<Tuple, typename Iterator::next>(
            entity, last_invocation_id, vals..., value);
      }
    } else {
      return std::make_tuple(vals...);
    }
  }

  template <class FilterIterator>
  std::vector<Entity> apply_filters(std::vector<Entity> entities,
                                    u_int64_t last_invocation_id) {
    if constexpr (FilterIterator::empty) {
      return entities;
    } else {
      std::vector<Entity> filtered;
      for (Entity e : entities) {
        if constexpr (ChangedConcept<typename FilterIterator::type>) {
          if (data.at(typeid(typename FilterIterator::type::component_type))
                  .at(e)
                  .first.has_changed(last_invocation_id))
            filtered.push_back(e);
        }
      }
      return apply_filters<typename FilterIterator::next>(filtered,
                                                          last_invocation_id);
    }
  }

  std::vector<u_int32_t>
  intersect_stores(std::vector<std::type_index> to_intersect) {
    std::vector<u_int32_t> entities;
    if (to_intersect.empty()) {
      return entities;
    }
    for (auto &p : data[to_intersect.back()]) {
      entities.push_back(p.first);
    }
    to_intersect.pop_back();
    std::vector<u_int32_t> intersected;
    for (auto e : entities) {
      bool bad = false;
      for (auto &i : to_intersect) {
        if (!data[i].contains(e)) {
          bad = true;
          break;
        }
      }
      if (!bad) {
        intersected.push_back(e);
      }
    }
    return intersected;
  }

  template <class T> T create_query(u_int64_t last_invocation) {
    using Tuple = typename decltype(T::entities)::value_type;
    using Iterator = typename T::component_type_iterator;
    std::vector<std::type_index> stores =
        Iterator::template get_type_indices<u_int32_t>();
    std::vector<Tuple> components;
    std::vector<u_int32_t> entities = intersect_stores(stores);
    entities =
        apply_filters<typename T::filter::iterator>(entities, last_invocation);
    for (u_int32_t e : entities) {
      components.push_back(query_tuple<Tuple, Iterator>(e, last_invocation));
    }
    T query;
    query.entities = std::move(components);
    return query;
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

public:
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
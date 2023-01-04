#pragma once

#include "Commands.hh"
#include "ComponentRegistry.hh"
#include "ComponentState.hh"
#include "ComponentStore.hh"
#include "Entity.hh"
#include "Filter.hh"
#include "Query.hh"
#include "World.hh"

#include "../InvocationID.hh"
#include "HypnoSquid.Core/Extension.hh"
#include "HypnoSquid.Core/components/ComponentExtension.hh"

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
  constexpr static ExtensionInvocationSchedule INVOCATION_SCHEDULE = AFTER_ALL_SYSTEMS;

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

  void invoke(InvocationID id, ExtensionInvocationSchedule schedule_state) {
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

  bool can_system_start(SystemState &system_state) {
    if (!requirements.contains(system_state.system_id))
      return true;
    auto &req = requirements[system_state.system_id];
    return std::ranges::all_of(req.mutable_components.begin(), req.mutable_components.end(),
                               [&](auto &c) {
                                 return !mutable_component_references.contains(c) &&
                                        const_component_reference_count[c] == 0;
                               }) &&
           std::ranges::all_of(req.const_components.begin(), req.const_components.end(),
                               [&](auto &c) { return !mutable_component_references.contains(c); });
  }

  void on_system_start(SystemState &system_state) {
    auto &req = requirements[system_state.system_id];
    mutable_component_references.insert(req.mutable_components.begin(), req.mutable_components.end());
    for (auto &m : req.const_components)
      const_component_reference_count[m]++;
  }

  void on_system_end(SystemState &system_state) {
    auto &req = requirements[system_state.system_id];
    for (auto &m : req.mutable_components)
      mutable_component_references.erase(m);
    for (auto &m : req.const_components)
      const_component_reference_count[m]--;
  }

  ComponentExtension() : world(component_registry) {}
};

} // namespace core
} // namespace hs
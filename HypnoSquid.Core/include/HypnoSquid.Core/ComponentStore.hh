#pragma once

#include "ComponentRegistry.hh"
#include "ComponentState.hh"
#include "Entity.hh"
#include "common.hh"
#include <unordered_set>

namespace hs {
namespace core {

class ComponentStore {
  std::unordered_map<ComponentID,
                     std::unordered_map<Entity, std::pair<std::unique_ptr<ComponentState>, unique_void_ptr>>>
      component_data;
  std::unordered_map<Entity, std::unordered_set<ComponentID>> entity_components;
  ComponentRegistry &component_registry;

public:
  ComponentStore(ComponentRegistry &registry) : component_registry(registry) {}

  void add_component(ComponentID cid, Entity entity, unique_void_ptr data, u_int32_t invocation_id) {
    if (!entity_components[entity].contains(cid)) {
      entity_components[entity].insert(cid);
      component_data[cid].emplace(
          entity, std::make_pair(std::make_unique<ComponentState>(ComponentState{.last_changed = invocation_id}),
                                 std::move(data)));
    }
  }

  void remove_component(ComponentID cid, Entity entity) {
    if (entity_components.contains(entity) && entity_components[entity].contains(cid)) {
      entity_components.erase(entity);
      component_data[cid].erase(entity);
      if (entity_components[entity].empty()) {
        entity_components.erase(entity);
      }
    }
  }

  bool has_component(ComponentID cid, Entity entity) const {
    return entity_components.contains(entity) && entity_components.at(entity).contains(cid);
  }

  void *get_component_data(ComponentID cid, Entity entity) {
    if (entity_components.contains(entity) && entity_components[entity].contains(cid)) {
      return component_data[cid].at(entity).second.get();
    }
    return nullptr;
  }

  ComponentState *get_component_state(ComponentID cid, Entity entity) {
    if (entity_components.contains(entity) && entity_components[entity].contains(cid)) {
      return component_data[cid].at(entity).first.get();
    }
    return nullptr;
  }

  const ComponentState *get_component_state(ComponentID cid, Entity entity) const {
    if (entity_components.contains(entity) && entity_components.at(entity).contains(cid)) {
      return component_data.at(cid).at(entity).first.get();
    }
    return nullptr;
  }

  template <class T> void add_component(Entity entity, unique_void_ptr data, u_int32_t invocation_id) {
    ComponentID cid = component_registry.template get_component_id<T>();
    add_component(cid, entity, std::move(data), invocation_id);
  }

  template <class T> void remove_component(Entity entity) {
    ComponentID cid = component_registry.template get_component_id<T>();
    remove_component(cid, entity);
  }

  template <class T> bool has_component(Entity entity) const {
    ComponentID cid = component_registry.template get_component_id<T>();
    return has_component(cid, entity);
  }

  template <class T> ComponentState *get_component_state(Entity e) {
    ComponentID cid = component_registry.template get_component_id<T>();
    return get_component_state(cid, e);
  }

  template <class T> const ComponentState *get_component_state(Entity e) const {
    ComponentID cid = component_registry.template get_component_id<T>();
    return get_component_state(cid, e);
  }

  template <class T> T *get_component_data(Entity e) {
    ComponentID cid = component_registry.template get_component_id<T>();
    return static_cast<T *>(get_component_data(cid, e));
  }
};

} // namespace core
} // namespace hs
#pragma once

#include "../common.hh"
#include "ComponentRegistry.hh"
#include "ComponentState.hh"
#include "Entity.hh"
#include <unordered_map>
#include <unordered_set>

namespace hs {
namespace core {

class AbstractComponentStore {
public:
  virtual ~AbstractComponentStore() = default;
  virtual void add_component(Entity, unique_void_ptr data, u_int32_t invocation_id) = 0;
  virtual void remove_component(Entity) = 0;
  [[nodiscard]] virtual bool has_component(Entity) const = 0;
  virtual ComponentState &get_component_state(Entity) = 0;
  [[nodiscard]] virtual const ComponentState &get_component_state(Entity) const = 0;
};

template <class T> class ComponentStore : public AbstractComponentStore {
  std::unordered_map<Entity, std::pair<ComponentState, T>> component_data;

public:
  void add_component(Entity entity, unique_void_ptr data, u_int32_t invocation_id) override {
    if (!component_data.contains(entity)) {
      component_data.template emplace(
          entity, std::make_pair(ComponentState{.last_changed = invocation_id}, *static_cast<T *>(data.get())));
    }
  }

  void remove_component(Entity entity) override { component_data.erase(entity); }

  [[nodiscard]] bool has_component(Entity entity) const override { return component_data.contains(entity); }

  T &get_component_data(Entity entity) { return component_data.at(entity).second; }

  ComponentState &get_component_state(Entity entity) override { return component_data.at(entity).first; }

  [[nodiscard]] const ComponentState &get_component_state(Entity entity) const override {
    return component_data.at(entity).first;
  }
};

} // namespace core
} // namespace hs
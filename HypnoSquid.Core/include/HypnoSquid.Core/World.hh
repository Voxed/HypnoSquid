#pragma once

#include "ComponentStore.hh"

namespace hs {
namespace core {

class World {
  std::unordered_map<ComponentID, std::unique_ptr<AbstractComponentStore>> stores;

  ComponentRegistry &registry;

public:
  explicit World(ComponentRegistry &registry) : registry(registry) {}

  AbstractComponentStore &get_component_store(ComponentID id) {
    if (!stores.contains(id)) {
      stores.emplace(id, registry.create_component_store(id));
    }
    return *stores.at(id);
  }

  template <class T> ComponentStore<std::remove_const_t<T>> &get_component_store() {
    return dynamic_cast<ComponentStore<std::remove_const_t<T>> &>(
        get_component_store(registry.template get_component_id<std::remove_const_t<T>>()));
  }

  [[nodiscard]] bool has_component_store(ComponentID id) const { return stores.contains(id); }
  [[nodiscard]] const AbstractComponentStore &get_component_store(ComponentID id) const { return *stores.at(id); }
};

} // namespace core
} // namespace hs
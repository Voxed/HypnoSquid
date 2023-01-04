#pragma once

#include "ComponentStore.hh"
#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_map>

namespace hs {
namespace core {

using ComponentID = u_int32_t;

/*
 * Attaches a unique id to every component based on their ComponentName.
 * O(1) speed complexity if I'm correct.
 */
class ComponentRegistry {

  std::unordered_map<std::string, ComponentID> component_ids;
  std::unordered_map<ComponentID, std::unique_ptr<AbstractComponentStore> (*)()> store_factories;
  ComponentID next_component_id = 1;

public:
  template <class Component> ComponentID get_component_id() {
    static ComponentID id = 0;
    if (id == 0) {
      std::string cid = Component::ID.get_id();
      if (component_ids.contains(cid)) {
        id = component_ids[cid];
        std::cout << "Component with cid " << cid << " aka. " << typeid(Component).name() << " with hash "
                  << typeid(Component).hash_code() << " fetched to id " << id << "." << std::endl;
      } else {
        id = next_component_id++;
        component_ids[cid] = id;
        std::cout << "Component with cid " << cid << " aka. " << typeid(Component).name() << " with hash "
                  << typeid(Component).hash_code() << " registered to id " << id << "." << std::endl;
        store_factories.emplace(id, []() -> std::unique_ptr<AbstractComponentStore> {
          return std::make_unique<ComponentStore<std::remove_const_t<Component>>>();
        });
      }
    }
    return id;
  }

  template <class Component>
    requires std::is_const_v<Component>
  ComponentID get_component_id() {
    return get_component_id<std::remove_const_t<Component>>();
  }

  std::unique_ptr<AbstractComponentStore> create_component_store(ComponentID id) { return store_factories.at(id)(); }
};

} // namespace core
} // namespace hs
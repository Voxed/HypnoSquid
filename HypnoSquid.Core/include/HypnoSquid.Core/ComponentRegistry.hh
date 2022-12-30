#pragma once

#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_map>

namespace hs {
namespace core {

/*
 * Attaches a unique id to every component based on their CID.
 * O(1) speed complexity if I'm correct.
 */
class ComponentRegistry {

  std::unordered_map<std::string, u_int32_t> component_ids;
  u_int32_t next_component_id = 1;

public:
  template <class Component> u_int32_t get_component_id() {
    static u_int32_t id = 0;
    if (id == 0) {
      std::string cid = Component::ID.get_id();
      if (component_ids.contains(cid)) {
        id = component_ids[cid];
        std::cout << "Component with cid " << cid << " aka. "
                  << typeid(Component).name() << " with hash "
                  << typeid(Component).hash_code() << " fetched to id " << id
                  << "." << std::endl;
      } else {
        id = next_component_id++;
        component_ids[cid] = id;
        std::cout << "Component with cid " << cid << " aka. "
                  << typeid(Component).name() << " with hash "
                  << typeid(Component).hash_code() << " registered to id " << id
                  << "." << std::endl;
      }
    }
    return id;
  }
};

} // namespace core
} // namespace hs
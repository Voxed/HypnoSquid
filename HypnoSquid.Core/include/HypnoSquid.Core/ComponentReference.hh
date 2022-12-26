#pragma once

#include "ComponentState.hh"
#include "SystemState.hh"
#include <cstdlib>
#include <type_traits>

namespace hs {
namespace core {

namespace concepts {
template <class T>
concept ComponentReference = requires { T::is_component_reference; };
}

template <class T> class ComponentReference {
  T *data = nullptr;
  ComponentState &component_state;
  SystemState system_state;

public:
  explicit ComponentReference(T *data, ComponentState &component_state,
                              SystemState system_state)
      : data(data), component_state(component_state),
        system_state(system_state) {}
  ComponentReference() = default;

  using value_type = std::remove_const_t<T>;
  static constexpr bool is_component_reference = true;

  [[nodiscard]] bool has_changed() const {
    return component_state.has_changed(system_state);
  }

  const value_type *get() const { return data; }

  value_type *get_mut() {
    component_state.last_changed = system_state.invocation_id;
    return data;
  }

  const value_type *operator->() const { return get(); }
};

} // namespace core
} // namespace hs
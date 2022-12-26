#pragma once

#include "ComponentState.hh"
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
  u_int64_t invocation_id = 0;
  u_int64_t last_invocation = 0;

public:
  explicit ComponentReference(T *data, ComponentState &component_state,
                              u_int64_t invocation_id,
                              u_int64_t last_invocation)
      : data(data), component_state(component_state),
        invocation_id(invocation_id), last_invocation(last_invocation) {}
  ComponentReference() = default;

  using value_type = std::remove_const_t<T>;
  static constexpr bool is_component_reference = true;

  [[nodiscard]] bool has_changed() const {
    return component_state.has_changed(last_invocation);
  }

  const value_type *get() const { return data; }

  value_type *get_mut() {
    component_state.last_changed = invocation_id;
    return data;
  }

  const value_type *operator->() const { return get(); }
};

} // namespace core
} // namespace hs
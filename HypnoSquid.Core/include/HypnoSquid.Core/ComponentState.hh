#pragma once

#include "SystemState.hh"
#include <cstdlib>

namespace hs {
namespace core {

struct ComponentState {
  u_int64_t last_changed = 0;

  [[nodiscard]] bool has_changed(SystemState system_state) const {
    return system_state.last_invocation_id < last_changed;
  }
};

} // namespace core
} // namespace hs
#pragma once

#include <cstdlib>

namespace hs {
namespace core {

struct ComponentState {
  u_int64_t last_changed = 0;

  [[nodiscard]] bool has_changed(u_int64_t last_invocation) const {
    return last_invocation <= last_changed;
  }
};

} // namespace core
} // namespace hs
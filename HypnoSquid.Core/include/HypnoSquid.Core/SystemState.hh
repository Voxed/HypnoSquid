#pragma once

#include <cstdlib>

namespace hs {
namespace core {

struct SystemState {
  u_int64_t last_invocation_id = 0;
  u_int64_t invocation_id = 0;
};

} // namespace core
} // namespace hs
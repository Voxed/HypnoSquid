#pragma once

#include <cstdlib>

namespace hs {
namespace core {

using SystemID = u_int32_t;

struct SystemState {
  SystemID system_id = 0;
  u_int64_t last_invocation_id = 0;
  u_int64_t invocation_id = 0;
};

} // namespace core
} // namespace hs
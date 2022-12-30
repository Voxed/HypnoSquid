#pragma once

#include <cstdlib>
#include <mutex>

namespace hs {
namespace core {

using Entity = u_int32_t;

// Atomic entity counter, to allow for immediate id allocation in concurrent
// command queues.
class EntityFactory {
  u_int32_t next_entity_id = 0;
  std::mutex mut;

public:
  u_int32_t create_entity() {
    mut.lock();
    u_int32_t id = next_entity_id++;
    mut.unlock();
    return id;
  }
};

} // namespace core
} // namespace hs
#pragma once

#include <cstdlib>
#include <type_traits>
#include <typeindex>
#include <vector>

namespace hs {
namespace core {

enum CommandType { REMOVE, ADD };

struct Command {
  CommandType type;
  union {
    struct {
      u_int32_t entity_id;
      std::type_index component_type;
      void *component_data;
      void (*component_destructor)(void const *);
    } add;
    struct {
      u_int32_t entity_id;
      std::type_index component_type;
    } remove;
  } action;
};

struct CommandBuffer {
  std::vector<Command> commands;
};

class Commands {
  CommandBuffer &buffer;

public:
  explicit Commands(CommandBuffer &buffer) : buffer(buffer) {}

  template <typename T> void add_component(u_int32_t entity_id, T data = T()) {
    Command cmd = {
        .type = ADD,
        .action = {.add = {.entity_id = entity_id,
                           .component_type = typeid(T),
                           .component_data = new T(data),
                           .component_destructor = [](void const *ptr) {
                             delete static_cast<T const *>(ptr);
                           }}}};
    buffer.commands.push_back(cmd);
  }

  template <typename T> void remove_component(u_int32_t entity_id) {
    Command cmd = {.type = REMOVE,
                   .action = {.remove = {.entity_id = entity_id,
                                         .component_type = typeid(T)}}};
    buffer.commands.push_back(cmd);
  }
};

} // namespace core
} // namespace hs
#pragma once

#include <string>

namespace hs {
namespace core {

struct PID {
  const char *plugin_name;

  constexpr PID(const char *plugin_name) : plugin_name(plugin_name) {}

  [[nodiscard]] std::string get_id() const { return plugin_name; }
};

struct CID {
  PID pid;
  const char *component_name;

  constexpr CID(PID pid, const char *component_name) : pid(pid), component_name(component_name) {}

  [[nodiscard]] std::string get_id() const { return pid.get_id() + "-" + component_name; }
};

} // namespace core
} // namespace hs
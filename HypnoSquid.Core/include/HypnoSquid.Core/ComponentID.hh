#pragma once

#include <string>

namespace hs {
namespace core {

struct PID {
  const char *plugin_name;

  constexpr explicit PID(const char *plugin_name) : plugin_name(plugin_name) {}

  [[nodiscard]] std::string get_id() const { return plugin_name; }
};

struct CID {
  const char *plugin_name;
  const char *component_name;

  constexpr CID(PID pid, const char *component_name) : plugin_name(pid.plugin_name), component_name(component_name) {}
  constexpr CID(const char *plugin_name, const char *component_name)
      : plugin_name(plugin_name), component_name(component_name) {}

  [[nodiscard]] std::string get_id() const { return std::string(plugin_name) + "-" + component_name; }
};

#define HS_STRINGIFY_MACRO(p) HS_STRINGIFY(p)
#define HS_STRINGIFY(p) #p
#define HS_COMPONENT(ComponentName) static constexpr hs::core::CID ID{HS_STRINGIFY_MACRO(HS_PLUGIN), #ComponentName};

} // namespace core
} // namespace hs
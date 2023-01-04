#pragma once

#include "../PluginName.hh"
#include <string>

namespace hs {
namespace core {

struct ComponentName {
  const char *plugin_name;
  const char *component_name;

  constexpr ComponentName(PluginName pid, const char *component_name)
      : plugin_name(pid.plugin_name), component_name(component_name) {}

  [[nodiscard]] std::string get_id() const { return std::string(plugin_name) + "-" + component_name; }
};

namespace concepts {

template <class T>
concept Component = requires {
                      { T::NAME } -> std::convertible_to<ComponentName>;
                    };

}

} // namespace core
} // namespace hs
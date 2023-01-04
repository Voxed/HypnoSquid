#pragma once

#include <string>
namespace hs {
namespace core {

struct PluginName {
  const char *plugin_name;

  constexpr explicit PluginName(const char *plugin_name) : plugin_name(plugin_name) {}

  [[nodiscard]] std::string get_id() const { return plugin_name; }
};

} // namespace core
} // namespace hs
#pragma once

#include <string>

namespace hs {
namespace core {

struct CID {
  const char *plugin_name;
  const char *component_name;

  constexpr CID(const char *plugin_name, const char *component_name)
      : plugin_name(plugin_name), component_name(component_name) {}

  [[nodiscard]] std::string get_id() const { return std::string(plugin_name) + "-" + component_name; }
};

#define HS_STRINGIFY_MACRO(p) HS_STRINGIFY(p)
#define HS_STRINGIFY(p) #p
#define HS_COMPONENT(ComponentName) static constexpr hs::core::CID ID{HS_STRINGIFY_MACRO(HS_PLUGIN), #ComponentName};

} // namespace core
} // namespace hs
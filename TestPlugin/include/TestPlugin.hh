#pragma once

#include "HypnoSquid.Core/world_extension/ComponentName.hh"
#include <string>

#undef HS_PLUGIN
#define HS_PLUGIN TestPlugin

constexpr hs::core::PluginName TestPlugin{"Test"};

struct TestPluginC {
  constexpr static hs::core::ComponentName ID{TestPlugin, "TestPluginC"};
};

class TestPlugin {};

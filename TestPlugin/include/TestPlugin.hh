#pragma once

#include "HypnoSquid.Core/components/ComponentID.hh"
#include <string>

#undef HS_PLUGIN
#define HS_PLUGIN TestPlugin

constexpr hs::core::PID TestPlugin{"Test"};

struct TestPluginC {
  HS_COMPONENT(TestPluginC)
};

class TestPlugin {};

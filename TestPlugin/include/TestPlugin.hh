#pragma once

#include "HypnoSquid.Core/ComponentID.hh"

#include <string>

#undef HS_PLUGIN
#define HS_PLUGIN TestPlugin

struct TestPluginC {
  HS_COMPONENT(TestPluginC)
};

class TestPlugin {};

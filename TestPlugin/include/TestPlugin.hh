#pragma once

#include "HypnoSquid.Core/ComponentID.hh"

#include <string>

constexpr auto TestPlugin = hs::core::PID("TestPlugin");

struct TestPluginC {
  static constexpr hs::core::CID ID = {TestPlugin, "TestPluginC"};
};

class TestPlugin {};

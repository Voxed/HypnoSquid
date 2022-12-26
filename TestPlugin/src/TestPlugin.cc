#include "../include/TestPlugin.hh"
#include "HypnoSquid.Core/Engine.hh"

void test(hs::core::EntityFactory &ef) {
  std::cout << "Plugin hooked!" << std::endl;
}

extern "C" void init_plugin(hs::core::Engine &engine) {
  std::cout << "Voxed: Hello! Thank you for using my awesome new plugin :)"
            << std::endl;

  engine.add_system(test);
}
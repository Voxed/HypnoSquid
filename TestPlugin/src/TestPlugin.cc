#include "../include/TestPlugin.hh"
#include "HypnoSquid.Core/Engine.hh"

void test(hs::core::EntityFactory &ef) {
  std::cout << "Voxed: Hello! Thank you for using my awesome new plugin :)"
            << std::endl;
}

extern "C" void init_plugin(hs::core::Engine &engine) {
  engine.add_startup_system(test);
}
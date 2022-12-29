#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include "HypnoSquid.Core/Engine.hh"

using namespace hs::core;
using namespace ::filters;

struct TestData {
  u_int32_t a;
};

struct TestData2 {
  u_int32_t b;
};

struct TestData3 {};

void test(Commands commands, EntityFactory &entity_factory) {
  for (u_int32_t i = 0; i < 2; i++) {
    u_int32_t e = entity_factory.create_entity();
    commands.add_component<TestData>(e, {.a = i});
    if (i == 0 || i == 1) {
      commands.add_component<TestData2>(e);
      if (i == 1) {
        commands.add_component<TestData3>(e);
      }
    } else {
      commands.add_component<TestData3>(e);
    }
  }
}

void test2(Query<TestData, Entity, const TestData2> &query,
           Query<TestData> &query2) {
  for (auto t : query.iter()) {
    auto &v = get<0>(t);
    if (get<1>(t) == 52)
      v.get_mut()->a;
    auto &v2 = get<2>(t);
  }
}

void test3(Query<All<Changed<TestData>, Not<TestData3>>, TestData, Entity> &q) {
  std::cout << "New" << std::endl;
  for (auto t : q.iter()) {
    auto c = get<0>(t);
    std::cout << "CHANGED!" << c->a << ":" << get<1>(t) << std::endl;
  }
}

// Temporary system to stop engine from flying away.
void sleep_system(EntityFactory &ef) {
  std::this_thread::sleep_for(std::chrono::milliseconds(16));
}

int main() {
  hs::core::Engine engine;
  engine.add_system(test)
      .add_system(test3)
      .add_system(test2)
      // Synchronised systems will run synchronised at the end of each
      // update.
      .add_synchronised_system(sleep_system)
      .run();

  return 0;
}
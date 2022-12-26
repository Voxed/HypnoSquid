#include <functional>
#include <iostream>

#include "Engine.hh"

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
  for (u_int32_t i = 0; i < 100; i++) {
    u_int32_t e = entity_factory.create_entity();
    commands.add_component<TestData>(e, {.a = i});
    if (i == 53 || i == 69) {
      commands.add_component<TestData2>(e);
      if (i == 53) {
        commands.add_component<TestData3>(e);
      }
    } else {
      commands.add_component<TestData3>(e);
    }
  }
}

void test2(Query<TestData, Entity, const TestData2> &query,
           Query<TestData> &query2) {
  for (auto t : query.entities) {
    auto &v = get<0>(t);
    if (get<1>(t) == 53)
      v.get_mut()->a;
    auto &v2 = get<2>(t);

    std::cout << v->a << ":" << get<1>(t) << std::endl;
  }
}

void test3(Query<All<Changed<TestData>, Not<TestData3>>, TestData> q) {
  for (auto t : q.entities) {
    auto c = get<0>(t);
    std::cout << "CHANGED!" << std::endl;
  }
}

int main() {
  hs::core::Engine engine;
  engine.add_system(test);
  engine.add_system(test2);
  engine.add_system(test3);
  engine.test();

  return 0;
}
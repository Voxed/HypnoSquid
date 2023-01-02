#include <functional>
#include <iostream>
#include <thread>

#include "HypnoSquid.Core/Engine.hh"
#include "TestPlugin/include/TestPlugin.hh"

using namespace hs::core;
using namespace ::filters;

#undef HS_PLUGIN
#define HS_PLUGIN Main

constexpr PID MainPlugin{"Main"};

struct TestData {
  HS_COMPONENT(TestData)
  u_int32_t a;
};

struct TestData2 {
  constexpr static CID ID{"Main", "TestData2"};
  u_int32_t b;
};

struct TestData3 {
  constexpr static CID ID{MainPlugin, "TestData2"};
};

struct TestData4 {
  HS_COMPONENT(TestData4)
};

void sys_a(Query<const TestData> q, Commands cmd, EntityFactory &ef) {
  std::cout << "StartA" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::cout << "EndA" << std::endl;
  cmd.add_component<TestData>(ef.create_entity());
}

void sys_b(Query<Changed<TestData>, const TestData, Entity> q, Commands cmd) {
  std::cout << "StartB" << std::endl;
  for (auto &t : q.iter()) {
    if (get<0>(t)->a > 3) {
      cmd.remove_component<TestData>(get<1>(t));
    } else {
      std::cout << get<0>(t)->a << "-" << get<1>(t) << std::endl;
    }
  }
  std::cout << "EndB" << std::endl;
}

// This system needs mutable access, so it should always be exclusive from sys_a and sys_b
void sys_c(Query<TestData> q) {
  std::cout << "StartC" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  for (auto &t : q.iter()) {
    get<0>(t).get_mut().a += 1;
  }
  std::cout << "EndC" << std::endl;
}

// This system needs some other component, so it should run asynchronous to sys_c
void sys_d(const Query<TestData2> &q, const Query<const TestData2> &q2) {
  std::cout << "StartD" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << "EndD" << std::endl;
}

// Temporary system to stop engine from flying away.
void sleep_system(EntityFactory &ef) { std::this_thread::sleep_for(std::chrono::milliseconds(16)); }

int main() {
  hs::core::Engine engine;
  engine.add_system(sys_a)
      .add_system(sys_b)
      .add_system(sys_c)
      .add_system(sys_d)
      .add_synchronised_system(sleep_system)
      .run();

  return 0;
}
#include <functional>
#include <iostream>
#include <thread>

#include "TestPlugin/include/TestPlugin.hh"
#include <HypnoSquid.Core/Engine.hh>

using namespace hs::core;
using namespace ::filters;

constexpr PluginName MainPlugin{"Main"};

struct TestData {
  constexpr static ComponentName NAME{MainPlugin, "TestData"};
  u_int32_t a;
};
static_assert(concepts::Component<TestData>); // If you want to ensure the component is correctly defined.

struct TestData2 {
  constexpr static ComponentName NAME{MainPlugin, "TestData2"};
  u_int32_t b;
};

struct TestData3 {
  constexpr static ComponentName NAME{MainPlugin, "TestData3"};
};

struct TestData4 {
  constexpr static ComponentName NAME{MainPlugin, "TestData4"};
};

struct Particle {
  constexpr static ComponentName NAME{MainPlugin, "Particle"};
  u_int32_t x;
  u_int32_t y;
};

struct MovingParticle {
  constexpr static ComponentName NAME{MainPlugin, "MovingParticle"};
};

void physics(Query<Particle, const MovingParticle, Entity> p, Query<Not<MovingParticle>, Particle> q, Commands cmds) {
  std::vector<std::pair<u_int32_t, u_int32_t>> colliders;
  for (auto &t : q.iter()) {
    const Particle &par = get<0>(t).get();
    colliders.emplace_back(par.x, par.y);
  }

  if (p.first()) {
    const Particle &par = get<0>(p.first().value()).get();
    std::pair<u_int32_t, u_int32_t> next_position = std::make_pair(par.x, par.y + 1);
    bool left = true, right = true, down = true;
    if (next_position.second >= 10)
      left = false, right = false, down = false;
    else
      for (const auto &c : colliders) {
        if (c.first == next_position.first && c.second == next_position.second)
          down = false;
        else if (c.first == next_position.first - 1 && c.second == next_position.second)
          left = false;
        else if (c.first == next_position.first + 1 && c.second == next_position.second)
          right = false;
      }
    if (down) {
      get<0>(p.first().value()).get_mut().y += 1;
    } else if (left) {
      get<0>(p.first().value()).get_mut().y += 1;
      get<0>(p.first().value()).get_mut().x -= 1;
    } else if (right) {
      get<0>(p.first().value()).get_mut().y += 1;
      get<0>(p.first().value()).get_mut().x += 1;
    } else {
      cmds.remove_component<MovingParticle>(get<2>(p.first().value()));
    }
  }
}

void render(Query<const Particle> q) {
  std::vector<std::pair<u_int32_t, u_int32_t>> colliders;
  for (auto &t : q.iter()) {
    const Particle &p = get<0>(t).get();
    colliders.emplace_back(p.x, p.y);
  }
  std::cout << std::endl;
  for (int r = 0; r < 10; r++) {
    for (int co = 0; co < 11; co++) {
      bool found = false;
      for (auto &c : colliders) {
        if (c.second == r && c.first == co) {
          std::cout << "#";
          found = true;
          break;
        }
      }
      if (!found) {
        std::cout << ".";
      }
    }
    std::cout << std::endl;
  }
}

void spawner(Query<const MovingParticle> p, Commands cmds, EntityFactory &ef) {
  if (!p.first()) {
    auto e = ef.create_entity();
    cmds.add_component<Particle>(e, {.x = 5, .y = 0});
    cmds.add_component<MovingParticle>(e);
  }
}

// Temporary system to stop engine from flying away.
void sleep_system(EntityFactory &ef) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

int main() {
  hs::core::Engine engine;
  engine.add_system(spawner)
      .add_system(physics)
      .add_synchronised_system(render)
      .add_synchronised_system(sleep_system)
      .run();

  return 0;
}
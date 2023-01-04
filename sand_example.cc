#include <iostream>

#include <HypnoSquid.Core/Engine.hh>

using namespace hs::core;
using namespace ::filters;

constexpr PluginName MainPlugin{"Main"};

struct Particle {
  constexpr static ComponentName NAME{MainPlugin, "Particle"};
  u_int32_t x;
  u_int32_t y;
};

struct MovingParticle {
  constexpr static ComponentName NAME{MainPlugin, "MovingParticle"};
};

void physics(Query<Particle, const MovingParticle, Entity> p, Query<Not<MovingParticle>, Particle> q, Commands cmds) {
  if (p.first()) {
    const Particle &par = get<0>(p.first().value()).get();
    std::pair<u_int32_t, u_int32_t> next_position = std::make_pair(par.x, par.y + 1);
    bool left = true, right = true, down = true;
    if (next_position.second >= 10) {
      cmds.remove_component<MovingParticle>(get<2>(p.first().value()));
    } else {
      for (const auto &qp : q.iter()) {
        auto &c = get<0>(qp).get();
        if (c.x == next_position.first && c.y == next_position.second)
          down = false;
        else if (c.x == next_position.first - 1 && c.y == next_position.second)
          left = false;
        else if (c.x == next_position.first + 1 && c.y == next_position.second)
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
}

void render(Query<const Particle> q) {
  std::cout << std::endl;
  for (int r = 0; r < 10; r++) {
    for (int co = 0; co < 41; co++) {
      bool found = false;
      for (auto &qp : q.iter()) {
        auto &c = get<0>(qp).get();
        if (c.y == r && c.x == co) {
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

// The spawner will actually run parallel to physics, seeing as they both only require constant access to
// MovingParticle.
void spawner(Query<const MovingParticle> p, Commands cmds, EntityFactory &ef) {
  if (!p.first()) {
    auto e = ef.create_entity();
    cmds.add_component<Particle>(e, {.x = 20, .y = 0});
    cmds.add_component<MovingParticle>(e);
  }
}

void sleep_system(EntityFactory &ef) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); }

int main() {
  hs::core::Engine engine;
  engine.add_system(spawner)
      .add_system(physics)
      .add_synchronised_system(render) // Needs to be synchronised to avoid jittering.
      .add_synchronised_system(sleep_system)
      .run();

  return 0;
}
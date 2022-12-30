#pragma once

#include "ComponentReference.hh"
#include "Filter.hh"
#include <functional>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hs {
namespace core {

template <class T> struct create_component_reference {
  using type = ComponentReference<T>;
};

template <class T> struct create_component_reference<const T> {
  using type = const ComponentReference<std::remove_const_t<T>>;
};

template <> struct create_component_reference<u_int32_t> {
  using type = u_int32_t;
};

template <class T>
using create_component_reference_t =
    typename create_component_reference<T>::type;

union QueryBufferItem {
  u_int32_t entity_id;
  struct {
    void *data;
    ComponentState &state;
  } component;
};

enum QueryBufferLayoutItemType { ENTITY, COMPONENT };

struct QueryBufferLayoutItem {
  QueryBufferLayoutItemType type;
  union {
    u_int32_t component_type;
  } data;

  QueryBufferLayoutItem
  from_query_parameter(std::type_identity<u_int32_t>,
                       ComponentRegistry &component_registry) {
    return QueryBufferLayoutItem{.type = ENTITY};
  };

  template <template <class> class T, class Arg>
  static QueryBufferLayoutItem
  from_query_parameter(std::type_identity<T<Arg>>,
                       ComponentRegistry &component_registry) {
    return QueryBufferLayoutItem{
        .type = COMPONENT,
        .data = {.component_type =
                     component_registry.template get_component_id<Arg>()}};
  }
};

struct QueryBuffer {
  std::vector<QueryBufferLayoutItem> layout;
  std::unordered_map<u_int32_t, std::vector<QueryBufferItem>> items;
  u_int64_t last_changed = 0;

  template <template <class...> class Query, class... Args>
  static QueryBuffer from_query(std::type_identity<Query<Args...>>,
                                ComponentRegistry &component_registry) {
    QueryBuffer buffer;
    (
        [&]() {
          if constexpr (std::is_same_v<Args, u_int32_t>) {
            buffer.layout.push_back(QueryBufferLayoutItem{.type = ENTITY});
          } else {
            buffer.layout.push_back(QueryBufferLayoutItem::from_query_parameter(
                std::type_identity<create_component_reference<Args>>(),
                component_registry));
          }
        }(),
        ...);
    return buffer;
  }

  template <template <class...> class Query, concepts::Filter Filter,
            class... Args>
  static QueryBuffer from_query(std::type_identity<Query<Filter, Args...>>,
                                ComponentRegistry &component_registry) {
    return from_query(std::type_identity<Query<Args...>>(), component_registry);
  }
};

template <class... Components> class QueryBase {
protected:
  QueryBuffer &buffer;
  SystemState &system_state;
  u_int64_t last_updated = 0;
  std::unordered_map<u_int32_t,
                     std::tuple<create_component_reference_t<Components>...>>
      entities;

  QueryBase(QueryBuffer &buffer, SystemState &system_state)
      : buffer(buffer), system_state(system_state) {}

  u_int32_t create(std::type_identity<u_int32_t>, const QueryBufferItem &item) {
    return item.entity_id;
  }

  template <template <class> class T, class Arg>
    requires concepts::ComponentReference<T<Arg>>
  T<Arg> create(std::type_identity<T<Arg>>, const QueryBufferItem &item) {
    return T(static_cast<Arg *>(item.component.data), item.component.state,
             system_state);
  }

  void update() {
    if (this->last_updated <= this->buffer.last_changed) {
      entities.clear();
      for (auto &p : buffer.items) {
        int idx = 0;
        entities.emplace(p.first,
                         std::move(std::make_tuple(create(
                             std::type_identity<std::remove_const_t<
                                 create_component_reference_t<Components>>>(),
                             p.second.at(([&]() { return idx++; }())))...)));
      }
    }
  }
};

template <class... Components> class Query : QueryBase<Components...> {
public:
  Query(QueryBuffer &buffer, SystemState &system_state)
      : QueryBase<Components...>(buffer, system_state) {}

  std::vector<std::tuple<create_component_reference_t<Components>...>> iter() {
    this->update();

    std::vector<std::tuple<create_component_reference_t<Components>...>> e;
    for (auto &p : this->entities) {
      e.push_back(p.second);
    }

    return e;
  }
};

// Annoying code duplication
template <concepts::Filter Filter, class... Components>
class Query<Filter, Components...> : QueryBase<Components...> {
  u_int64_t last_filtered = 0;
  std::function<std::unordered_set<u_int32_t>(std::unordered_set<u_int32_t>)>
      filter;
  std::vector<std::tuple<create_component_reference_t<Components>...>> filtered;

public:
  Query(QueryBuffer &buffer, SystemState &system_state,
        std::function<
            std::unordered_set<u_int32_t>(std::unordered_set<u_int32_t>)>
            filter)
      : filter(std::move(filter)), QueryBase<Components...>(buffer,
                                                            system_state) {}

  std::vector<std::tuple<create_component_reference_t<Components>...>> iter() {
    this->update();

    if (last_filtered <= this->system_state.invocation_id) {
      std::unordered_set<u_int32_t> e;
      for (auto &p : this->entities) {
        e.insert(p.first);
      }
      filtered.clear();
      for (auto &p : filter(e)) {
        filtered.push_back(this->entities.at(p));
      }
      last_filtered = this->system_state.invocation_id + 1;
    }

    return filtered;
  }
};

} // namespace core
} // namespace hs
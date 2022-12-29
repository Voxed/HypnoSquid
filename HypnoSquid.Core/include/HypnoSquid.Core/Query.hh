#pragma once

#include "ComponentReference.hh"
#include "Filter.hh"
#include <functional>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

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
    u_int32_t nothing; // TODO: Remove when getting rid of typeindex
    std::type_index component_type;
  } data;
  template <class T>
  static QueryBufferLayoutItem from_query_parameter(std::type_identity<T>);

  template <>
  QueryBufferLayoutItem
  from_query_parameter<u_int32_t>(std::type_identity<u_int32_t>) {
    return QueryBufferLayoutItem{.type = ENTITY};
  };

  template <template <class> class T, class Arg>
  static QueryBufferLayoutItem
  from_query_parameter(std::type_identity<T<Arg>>) {
    return QueryBufferLayoutItem{.type = COMPONENT,
                                 .data = {.component_type = typeid(Arg)}};
  }
};

struct QueryBuffer {
  std::vector<QueryBufferLayoutItem> layout;
  std::unordered_map<u_int32_t, std::vector<QueryBufferItem>> items;
  u_int64_t last_changed = 0;

  template <template <class...> class Query, class... Args>
  static QueryBuffer from_query(std::type_identity<Query<Args...>>) {
    QueryBuffer buffer;
    (
        [&]() {
          if constexpr (std::is_same_v<Args, u_int32_t>) {
            buffer.layout.push_back(QueryBufferLayoutItem{.type = ENTITY});
          } else {
            buffer.layout.push_back(QueryBufferLayoutItem::from_query_parameter(
                std::type_identity<create_component_reference<Args>>()));
          }
        }(),
        ...);
    return buffer;
  }

  template <template <class...> class Query, concepts::Filter Filter,
            class... Args>
  static QueryBuffer from_query(std::type_identity<Query<Filter, Args...>>) {
    return from_query(std::type_identity<Query<Args...>>());
  }
};

template <class... Components> struct Query {
  QueryBuffer &buffer;
  SystemState &system_state;
  u_int64_t last_updated = 0;
  std::vector<std::tuple<create_component_reference_t<Components>...>> entities;
  std::function<std::unordered_set<u_int32_t>(std::unordered_set<u_int32_t>)>
      filter;

  template <class T>
  T create(std::type_identity<T>, const QueryBufferItem &item);

  template <>
  u_int32_t create<u_int32_t>(std::type_identity<u_int32_t>,
                              const QueryBufferItem &item) {
    return item.entity_id;
  }

  template <template <class> class T, class Arg>
    requires concepts::ComponentReference<T<Arg>>
  T<Arg> create(std::type_identity<T<Arg>>, const QueryBufferItem &item) {
    return T(static_cast<Arg *>(item.component.data), item.component.state,
             system_state);
  }

  void update() {
    entities.clear();
    for (auto &p : buffer.items) {
      int idx = 0;
      entities.push_back(std::make_tuple(create(
          std::type_identity<
              std::remove_const_t<create_component_reference_t<Components>>>(),
          p.second.at(([&]() { return idx++; }())))...));
    }
  }

  std::vector<std::tuple<create_component_reference_t<Components>...>> iter() {
    if (last_updated <= buffer.last_changed) {
      update();
      last_updated = system_state.invocation_id;
    }

    return entities;
  }
};

// Annoying code duplication
template <concepts::Filter Filter, class... Components>
struct Query<Filter, Components...> {
  QueryBuffer &buffer;
  SystemState &system_state;
  u_int64_t last_updated = 0;
  u_int64_t last_filtered = 0;
  std::unordered_map<u_int32_t,
                     std::tuple<create_component_reference_t<Components>...>>
      entities;
  std::function<std::unordered_set<u_int32_t>(std::unordered_set<u_int32_t>)>
      filter;
  std::vector<std::tuple<create_component_reference_t<Components>...>> filtered;

  template <class T>
  T create(std::type_identity<T>, const QueryBufferItem &item);

  template <>
  u_int32_t create<u_int32_t>(std::type_identity<u_int32_t>,
                              const QueryBufferItem &item) {
    return item.entity_id;
  }

  template <template <class> class T, class Arg>
    requires concepts::ComponentReference<T<Arg>>
  T<Arg> create(std::type_identity<T<Arg>>, const QueryBufferItem &item) {
    return T(static_cast<Arg *>(item.component.data), item.component.state,
             system_state);
  }

  void update() {
    entities.clear();
    for (auto &p : buffer.items) {
      int idx = 0;
      entities.emplace(
          p.first, std::move(std::make_tuple(
                       create(std::type_identity<std::remove_const_t<
                                  create_component_reference_t<Components>>>(),
                              p.second.at(([&]() { return idx++; }())))...)));
    }
  }

  std::vector<std::tuple<create_component_reference_t<Components>...>> iter() {
    if (last_updated <= buffer.last_changed) {
      update();
      last_updated = system_state.invocation_id;
    }

    if (last_filtered <= system_state.invocation_id) {
      std::unordered_set<u_int32_t> e;
      for (auto &p : entities) {
        e.insert(p.first);
      }
      filtered.clear();
      for (auto &p : filter(e)) {
        filtered.push_back(entities.at(p));
      }
      last_filtered = system_state.invocation_id + 1;
    }

    return filtered;
  }
};

} // namespace core
} // namespace hs
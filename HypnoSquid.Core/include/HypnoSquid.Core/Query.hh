#pragma once

#include "ComponentReference.hh"
#include "Filter.hh"
#include <functional>
#include <optional>
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

template <class T> using create_component_reference_t = typename create_component_reference<T>::type;

union QueryBufferItem {
  struct {
    void *data;
    ComponentState &state;
  } component;
};

enum QueryBufferLayoutItemType { COMPONENT };

struct QueryBufferLayoutItem {
  QueryBufferLayoutItemType type;
  union {
    u_int32_t component_type;
  } data;

  template <template <class> class T, class Arg>
  static QueryBufferLayoutItem from_query_parameter(std::type_identity<T<Arg>>, ComponentRegistry &component_registry) {
    static_assert(!std::is_const_v<Arg>);
    return QueryBufferLayoutItem{.type = COMPONENT,
                                 .data = {.component_type = component_registry.template get_component_id<Arg>()}};
  }
};

struct QueryBuffer {
  std::vector<QueryBufferLayoutItem> layout;
  std::unordered_map<u_int32_t, std::vector<QueryBufferItem>> items;
  u_int64_t last_changed = 0;

  template <template <class...> class Query, class... Args>
  static QueryBuffer from_query(std::type_identity<Query<Args...>>, ComponentRegistry &component_registry) {
    QueryBuffer buffer;
    (
        [&]() {
          if constexpr (!std::is_same_v<Args, u_int32_t>) {
            buffer.layout.push_back(QueryBufferLayoutItem::from_query_parameter(
                std::type_identity<std::remove_const_t<create_component_reference_t<Args>>>(), component_registry));
          }
        }(),
        ...);
    return buffer;
  }

  template <template <class...> class Query, concepts::Filter Filter, class... Args>
  static QueryBuffer from_query(std::type_identity<Query<Filter, Args...>>, ComponentRegistry &component_registry) {
    return from_query(std::type_identity<Query<Args...>>(), component_registry);
  }

  template <class... Args> bool is_suitable(ComponentRegistry &component_registry) {
    bool suitable = true;
    // Check if all args are found in layout items
    (
        [&]() {
          if constexpr (!std::is_same_v<Args, u_int32_t>) {
            bool found_layout = false;
            for (auto &l : layout) {
              if (l.type == COMPONENT) {
                if (component_registry.template get_component_id<std::remove_const_t<Args>>() ==
                    l.data.component_type) {
                  found_layout = true;
                }
              }
            }
            if (!found_layout) {
              suitable = false;
            }
          }
        }(),
        ...);
    if (suitable) {
      // Check if all layout items are found in args
      for (auto &l : layout) {
        if (l.type == COMPONENT) {
          bool found_arg = false;
          (
              [&]() {
                if constexpr (!std::is_same_v<Args, u_int32_t>) {
                  if (component_registry.template get_component_id<std::remove_const_t<Args>>() ==
                      l.data.component_type) {
                    found_arg = true;
                  }
                }
              }(),
              ...);
          if (!found_arg) {
            suitable = false;
            break;
          }
        }
      }
    }
    return suitable;
  }
};

/*
 * A mapping between query buffer indices to query indices.
 * Facilitates query buffer reuse between multiple similar queries.
 */
struct QueryBufferMapping {
  std::unordered_map<size_t, size_t> mapping;
  template <class... Components>
  static QueryBufferMapping from(QueryBuffer &buffer, ComponentRegistry &component_registry) {
    QueryBufferMapping mapping;
    [&]<std::size_t... Is>(std::index_sequence<Is...>) {
      (
          [&]() {
            if constexpr (concepts::ComponentReference<Components>) {
              int layout_idx = 0;
              for (auto &layout_item : buffer.layout) {
                if (layout_item.type == COMPONENT) {
                  if (component_registry.template get_component_id<Components>() == layout_item.data.component_type) {
                    mapping.mapping[Is] = layout_idx;
                    break;
                  }
                }
                layout_idx += 1;
              }
            }
          }(),
          ...);
    }
    (std::make_index_sequence<sizeof...(Components)>{});

    for (auto &p : mapping.mapping) {
      std::cout << p.first << "->" << p.second << std::endl;
    }

    return mapping;
  }

  size_t map(size_t i) { return mapping[i]; }
};

template <class... Components> class QueryBase {
protected:
  QueryBuffer &buffer;
  SystemState &system_state;

  QueryBufferMapping buffer_mapping;
  const ComponentStore &store;

  u_int64_t last_updated = 0;
  std::unordered_map<u_int32_t, std::tuple<create_component_reference_t<Components>...>> entities;
  u_int64_t last_filtered = 0;
  std::vector<std::tuple<create_component_reference_t<Components>...>> filtered;

  QueryBase(QueryBuffer &buffer, SystemState &system_state, ComponentRegistry &component_registry,
            const ComponentStore &store)
      : buffer(buffer), system_state(system_state), store(store) {
    buffer_mapping = QueryBufferMapping::from<std::remove_const_t<Components>...>(buffer, component_registry);
  }

  u_int32_t create(std::type_identity<u_int32_t>, size_t, u_int32_t entity_id, const std::vector<QueryBufferItem> &) {
    return entity_id;
  }

  template <template <class> class T, class Arg>
    requires concepts::ComponentReference<T<Arg>>
  T<Arg> create(std::type_identity<T<Arg>>, size_t buffer_index, u_int32_t, const std::vector<QueryBufferItem> &b) {
    return T(static_cast<Arg *>(b.at(buffer_index).component.data),
             b.at(buffer_mapping.map(buffer_index)).component.state, system_state);
  }

  virtual std::unordered_set<Entity> filter(std::unordered_set<Entity> e) { return e; }

  void update() {
    if (this->last_updated <= this->buffer.last_changed) {
      entities.clear();
      for (auto &p : buffer.items) {
        int idx = 0;
        [&]<std::size_t... Is>(std::index_sequence<Is...>) {
          entities.emplace(
              p.first, std::move(std::make_tuple(
                           create(std::type_identity<std::remove_const_t<create_component_reference_t<Components>>>(),
                                  Is, p.first, p.second)...)));
        }
        (std::make_index_sequence<sizeof...(Components)>{});
      }
    }

    // Changed
    if (last_filtered < this->system_state.invocation_id) {
      std::unordered_set<u_int32_t> e;
      for (auto &p : this->entities) {
        e.insert(p.first);
      }
      filtered.clear();
      for (auto &p : filter(e)) {
        filtered.push_back(this->entities.at(p));
      }
      last_filtered = this->system_state.invocation_id;
    }
  }

public:
  std::vector<std::tuple<create_component_reference_t<Components>...>> iter() {
    this->update();

    return filtered;
  }

  std::optional<std::tuple<create_component_reference_t<Components>...>> first() {
    this->update();

    if (filtered.empty())
      return {};
    else
      return filtered.front();
  }
};

template <class... Components> class Query : public QueryBase<Components...> {
public:
  Query(QueryBuffer &buffer, SystemState &system_state, ComponentRegistry &component_registry,
        const ComponentStore &store)
      : QueryBase<Components...>(buffer, system_state, component_registry, store) {}
};

template <concepts::Filter Filter, class... Components>
class Query<Filter, Components...> : public QueryBase<Components...> {
  template <concepts::Filter... Filters>
  std::unordered_set<Entity> apply_all_filters(std::type_identity<filters::All<Filters...>>,
                                               std::unordered_set<Entity> entities, SystemState system_state) {
    std::unordered_set<Entity> filtered = std::move(entities);
    (
        [&]() {
          std::unordered_set<Entity> old_filtered = std::move(filtered);
          filtered.clear();
          for (const auto &e : apply_filter<Filters>(old_filtered, system_state))
            filtered.insert(e);
        }(),
        ...);
    return filtered;
  }

  template <concepts::Filter... Filters>
  std::unordered_set<Entity> apply_any_filters(std::type_identity<filters::All<Filters...>>,
                                               std::unordered_set<Entity> entities, SystemState system_state) {
    std::unordered_set<Entity> filtered;
    (
        [&]() {
          for (const auto &e : apply_filter<Filters>(entities, system_state))
            filtered.insert(e);
        }(),
        ...);
    return filtered;
  }

  template <concepts::Nop F> std::unordered_set<Entity> apply_filter(std::unordered_set<Entity> entities) {
    return entities;
  }

  template <concepts::Changed F> std::unordered_set<Entity> apply_filter(const std::unordered_set<Entity> &entities) {
    std::unordered_set<Entity> filtered;
    for (const Entity &e : entities)
      if (this->store.template get_component_state<typename F::component_type>(e)->has_changed(this->system_state))
        filtered.insert(e);
    return filtered;
  }

  template <concepts::Not F> std::unordered_set<Entity> apply_filter(const std::unordered_set<Entity> &entities) {
    std::unordered_set<Entity> filtered;
    for (const Entity &e : entities)
      if (!this->store.template has_component<typename F::component_type>(e))
        filtered.insert(e);
    return filtered;
  }

  template <concepts::All F> std::unordered_set<Entity> apply_filter(const std::unordered_set<Entity> &entities) {
    return apply_all_filters(std::type_identity<F>(), entities, this->system_state);
  }

  template <concepts::Any F> std::unordered_set<Entity> apply_filter(const std::unordered_set<Entity> &entities) {
    return apply_any_filters(std::type_identity<F>(), entities, this->system_state);
  }

  std::unordered_set<Entity> filter(std::unordered_set<Entity> e) override { return apply_filter<Filter>(e); }

public:
  Query(QueryBuffer &buffer, SystemState &system_state, ComponentRegistry &component_registry,
        const ComponentStore &store)
      : QueryBase<Components...>(buffer, system_state, component_registry, store) {}
};

} // namespace core
} // namespace hs
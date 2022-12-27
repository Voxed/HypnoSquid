#pragma once

#include "ComponentReference.hh"
#include "Filter.hh"
#include <type_traits>

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

template <class... Components> struct Query {
  std::vector<std::tuple<create_component_reference_t<Components>...>> entities;
};

template <concepts::Filter Filter, class... Components>
struct Query<Filter, Components...> {
  std::vector<std::tuple<create_component_reference_t<Components>...>> entities;
};

} // namespace core
} // namespace hs
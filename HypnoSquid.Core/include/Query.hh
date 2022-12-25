#pragma once

#include "ComponentReference.hh"
#include "Filter.hh"
#include "TypeIterator.hh"
#include <type_traits>

namespace hs {
namespace core {

namespace concepts {
template <class T>
concept QueryConcept = requires { T::is_query; };
}

template <class T> struct QueryTypeWrapper {
  using processed = T;
};

template <class T>
  requires(!std::is_const_v<std::remove_pointer_t<T>> &&
           !std::is_same_v<u_int32_t, T>)
struct QueryTypeWrapper<T> {
  using processed = ComponentReference<std::remove_pointer_t<T>>;
};

template <class T>
  requires(std::is_const_v<std::remove_pointer_t<T>> &&
           !std::is_same_v<u_int32_t, T>)
struct QueryTypeWrapper<T> {
  using processed = const ComponentReference<T>;
};

template <class... Components> struct Query {
  static constexpr bool is_query = true;
  using component_type_iterator =
      TypeIterator<0, typename QueryTypeWrapper<Components>::processed...>;
  std::vector<std::tuple<typename QueryTypeWrapper<Components>::processed...>>
      entities;
  using filter = Filter<>;
};

template <concepts::FilterConcept Filter, class... Components>
struct Query<Filter, Components...> {
  static constexpr bool is_query = true;
  using component_type_iterator =
      TypeIterator<0, typename QueryTypeWrapper<Components>::processed...>;
  std::vector<std::tuple<typename QueryTypeWrapper<Components>::processed...>>
      entities;
  using filter = Filter;
};

} // namespace core
} // namespace hs
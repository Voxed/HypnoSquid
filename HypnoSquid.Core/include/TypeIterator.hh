#pragma once

#include "common.hh"
#include <cstdlib>
#include <typeindex>
#include <vector>

namespace hs {
namespace core {

/*
 * Voxed's shitty template pack iterator (patent pending)
 */
template <int Index, class... Types> struct TypeIterator {};

template <int Index, class First, class... Types>
struct TypeIterator<Index, First, Types...> {
  using type = First;
  using next = TypeIterator<Index + 1, Types...>;
  static constexpr size_t index = Index;
  static constexpr bool empty = false;

  template <class... Except>
  static constexpr std::vector<std::type_index> get_type_indices() {
    std::vector<std::type_index> other =
        next::template get_type_indices<Except...>();
    if constexpr (!concepts::is_in<type, Except...>) {
      if constexpr (concepts::TypeDecorator<type>) {
        other.emplace_back(typeid(typename type::value_type));
      } else {
        other.emplace_back(typeid(std::remove_pointer_t<type>));
      }
    }
    return other;
  }
};

template <int Index> struct TypeIterator<Index> {
  static constexpr size_t index = Index;
  static constexpr bool empty = true;

  template <class... Except>
  static constexpr std::vector<std::type_index> get_type_indices() {
    return {};
  }
};

} // namespace core
} // namespace hs
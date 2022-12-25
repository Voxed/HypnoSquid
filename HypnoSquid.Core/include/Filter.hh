#pragma once

#include "TypeIterator.hh"

namespace hs {
namespace core {

namespace concepts {
template <class T>
concept FilterConcept = requires { T::is_filter; };
}

template <class... Filters> struct Filter {
  static constexpr bool is_filter = true;
  using iterator = TypeIterator<0, Filters...>;
};

template <class T>
concept ChangedConcept = requires { T::is_changed; };

namespace filters {

template <class Component> struct Changed {
  using component_type = Component;
  static constexpr bool is_changed = true;
};

} // namespace filters

} // namespace core
} // namespace hs
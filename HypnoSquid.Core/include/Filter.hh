#pragma once

#include "TypeIterator.hh"

namespace hs {
namespace core {

namespace concepts {
template <class T>
concept Filter = requires { T::is_filter; };
}
/*
template <class... Filters> struct Filter {
  static constexpr bool is_filter = true;
  using iterator = TypeIterator<0, Filters...>;
};
*/

namespace concepts {
template <class T>
concept Changed = requires { T::is_changed; };

template <class T>
concept Not = requires { T::is_not; };

template <class T>
concept Nop = requires { T::is_nop; };

template <class T>
concept All = requires { T::is_all; };

template <class T>
concept Any = requires { T::is_any; };
} // namespace concepts

namespace filters {

template <class Component> struct Changed {
  using component_type = Component;
  static constexpr bool is_changed = true;
  static constexpr bool is_filter = true;
};

template <class Component> struct Not {
  using component_type = Component;
  static constexpr bool is_not = true;
  static constexpr bool is_filter = true;
};

template <class... Filters> struct All {
  using iterator = TypeIterator<0, Filters...>;
  static constexpr bool is_filter = true;
  static constexpr bool is_all = true;
};

template <class... Filters> struct Any {
  using iterator = TypeIterator<0, Filters...>;
  static constexpr bool is_filter = true;
  static constexpr bool is_any = true;
};

struct Nop {
  static constexpr bool is_nop = true;
  static constexpr bool is_filter = true;
};

} // namespace filters

} // namespace core
} // namespace hs
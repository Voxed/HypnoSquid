#pragma once

#include "../common.hh"
#include <iostream>

namespace hs {
namespace core {

struct Filter {};

namespace filters {

template <class Component> struct Changed : Filter {
  using component_type = Component;
};

template <class Component> struct Not : Filter {
  using component_type = Component;
};

template <class... Filters> struct All : Filter {};

template <class... Filters> struct Any : Filter {};

struct Nop : Filter {};

} // namespace filters

namespace concepts {
template <class T>
concept Filter = std::is_base_of_v<Filter, T>;

template <class T>
concept Changed = specialization_of<filters::Changed, T>;

template <class T>
concept Not = specialization_of<filters::Not, T>;

template <class T>
concept Nop = std::is_same_v<filters::Nop, T>;

template <class T>
concept All = specialization_of<filters::All, T>;

template <class T>
concept Any = specialization_of<filters::Any, T>;
} // namespace concepts

} // namespace core
} // namespace hs
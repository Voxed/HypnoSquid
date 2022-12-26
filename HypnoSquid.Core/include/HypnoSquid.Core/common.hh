#pragma once

#include <type_traits>

namespace hs {
namespace core {

namespace concepts {
template <class T>
concept TypeDecorator = requires { typename T::value_type; };

template <class T, class... Ts>
concept is_in = std::disjunction_v<std::is_same<T, Ts>...>;
} // namespace concepts

} // namespace core
} // namespace hs
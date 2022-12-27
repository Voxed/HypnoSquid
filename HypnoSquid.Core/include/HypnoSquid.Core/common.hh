#pragma once

#include <type_traits>

namespace hs {
namespace core {

namespace concepts {
template <class T>
concept TypeDecorator = requires { typename T::value_type; };

template <class T, class... Ts>
concept is_in = std::disjunction_v<std::is_same<T, Ts>...>;

namespace {
template <template <typename...> typename, template <typename...> typename>
struct specialization_of_impl_s : std::false_type {};

template <template <typename...> typename T>
struct specialization_of_impl_s<T, T> : std::true_type {};

template <template <class...> class U, template <class...> class T,
          class... Args>
constexpr bool specialization_of_impl(std::type_identity<T<Args...>>) {
  return specialization_of_impl_s<T, U>::value;
}

} // namespace

template <template <class...> class U, class T>
concept specialization_of =
    specialization_of_impl<U>(std::type_identity<std::remove_const_t<T>>());

} // namespace concepts

} // namespace core
} // namespace hs
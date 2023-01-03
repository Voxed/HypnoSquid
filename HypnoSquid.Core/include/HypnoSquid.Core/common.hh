#pragma once

#include <memory>
#include <type_traits>

namespace hs {
namespace core {

template <class T> void simple_deleter(void const *ptr) { delete static_cast<T const *>(ptr); }

using unique_void_ptr = std::unique_ptr<void, void (*)(void const *)>;
template <class T> constexpr unique_void_ptr make_unique_void(T *ptr) {
  return unique_void_ptr(ptr, simple_deleter<T>);
}

namespace concepts {
template <class T, class... Ts>
concept is_in = std::disjunction_v<std::is_same<T, Ts>...>;

namespace {
template <template <typename...> typename, template <typename...> typename>
struct specialization_of_impl_s : std::false_type {};

template <template <typename...> typename T> struct specialization_of_impl_s<T, T> : std::true_type {};

template <template <class...> class U, template <class...> class T, class... Args>
constexpr bool specialization_of_impl(std::type_identity<T<Args...>>) {
  return specialization_of_impl_s<T, U>::value;
}

} // namespace

template <template <class...> class U, class T>
concept specialization_of = specialization_of_impl<U>(std::type_identity<std::remove_const_t<T>>());

} // namespace concepts

} // namespace core
} // namespace hs
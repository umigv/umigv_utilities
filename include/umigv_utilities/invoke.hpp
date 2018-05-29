#ifndef UMIGV_INVOKE_HPP
#define UMIGV_INVOKE_HPP

// backports std::invoke from C++17 to C++14

#include "umigv_utilities/traits.hpp"
#include "umigv_utilities/types.hpp"

#include <tuple>
#include <type_traits>
#include <utility>

namespace umigv {
namespace detail {

template <
    typename T, typename F, typename U, typename ...As,
    typename = std::enable_if_t<std::is_base_of<T, std::decay_t<U>>::value>
>
constexpr decltype(auto) invoke(F T::* f, U &&u, As &&...args)
noexcept(noexcept((std::forward<U>(u).*f)(std::forward<As>(args)...))) {
    return (std::forward<U>(u).*f)(std::forward<As>(args)...);
}

template <
    typename F, typename T, typename U, typename ...As,
    typename = std::enable_if_t<std::is_base_of<T, std::decay_t<U>>::value>
>
constexpr decltype(auto) invoke(F T::* f, U *u, As &&...args)
noexcept(noexcept((u->*f)(std::forward<As>(args)...))) {
    return (u->*f)(std::forward<As>(args)...);
}

template <typename F, typename ...As>
constexpr decltype(auto) invoke(F &&f, As &&...args)
noexcept(noexcept(std::forward<F>(f)(std::forward<As>(args)...))) {
    return std::forward<F>(f)(std::forward<As>(args)...);
}

template <typename F, typename ArgTuple, usize ...Is>
constexpr decltype(auto) apply(F &&f, ArgTuple &&t,
                               std::index_sequence<Is...>)
noexcept(noexcept(
    invoke(
        std::forward<F>(f),
        std::forward<std::tuple_element_t<Is, ArgTuple>>(
            std::get<Is>(std::forward<ArgTuple>(t))
        )...
    )
)) {
    return invoke(
        std::forward<F>(f),
        std::forward<std::tuple_element_t<Is, ArgTuple>>(
            std::get<Is>(std::forward<ArgTuple>(t))
        )...
    );
}

template <typename T>
using tuple_index_sequence =
    std::make_index_sequence<std::tuple_size<T>::value>;

template <typename F, typename ArgTuple, typename = void>
struct is_invocable : std::false_type { };

template <typename F, typename ArgTuple>
struct is_invocable<
    F,
    ArgTuple,
    void_t<decltype(apply(std::declval<F>(), std::declval<ArgTuple>(),
                          std::declval<tuple_index_sequence<ArgTuple>>()))>
> : std::true_type { };

template <typename F, typename ArgTuple, typename = void>
struct is_nothrow_invocable : std::false_type { };

template <typename F, typename ArgTuple>
struct is_nothrow_invocable<
    F,
    ArgTuple,
    void_t<decltype(apply(std::declval<F>(), std::declval<ArgTuple>(),
                          std::declval<tuple_index_sequence<ArgTuple>>()))>
> : true_type_if_t<
    noexcept(apply(std::declval<F>(), std::declval<ArgTuple>(),
                   std::declval<tuple_index_sequence<ArgTuple>>()))
> { };

template <typename F, typename ArgTuple,
          bool IsInvocable = is_invocable<F, ArgTuple>::value>
struct invoke_result { };

template <typename F, typename ArgTuple>
struct invoke_result<F, ArgTuple, true> {
    using type =
        decltype((apply(std::declval<F>(), std::declval<ArgTuple>(),
                        std::declval<tuple_index_sequence<ArgTuple>>())));
};

} // namespace detail

template <typename F, typename ...As>
struct is_invocable : detail::is_invocable<F, std::tuple<As...>> { };

template <typename F, typename ...As>
constexpr bool is_invocable_v = is_invocable<F, As...>::value;

template <typename F, typename ...As>
struct is_nothrow_invocable
: detail::is_nothrow_invocable<F, std::tuple<As...>> { };

template <typename F, typename ...As>
constexpr bool is_nothrow_invocable_v = is_nothrow_invocable<F, As...>::value;

template <typename F, typename ...As>
using invoke_result = detail::invoke_result<F, std::tuple<As...>>;

template <typename F, typename ...As>
using invoke_result_t = typename invoke_result<F, As...>::type;

template <typename F, typename ...As,
          std::enable_if_t<is_invocable_v<F, As...>, int> = 0>
constexpr invoke_result_t<F, As...> invoke(F &&f, As &&...args)
noexcept(is_nothrow_invocable_v<F, As...>) {
    return detail::invoke(std::forward<F>(f), std::forward<As>(args)...);
}

} // namespace umigv

#endif

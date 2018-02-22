#ifndef UMIGV_DETAIL_INVOKE_HPP
#define UMIGV_DETAIL_INVOKE_HPP

// packports std::invoke from C++17 to C++14

#include <type_traits> // std::enable_if_t
#include <utility> // std::declval, std::forward

namespace umigv {
namespace detail {

template <typename T, typename Function, typename U, typename ...Args,
          typename = std::enable_if_t<std::is_base_of<T, std::decay_t<U>>::value>>
decltype(auto) invoke_impl(Function T::* function, U &&u, Args &&...args)
    noexcept(noexcept((std::declval<U>().*function)(std::declval<Args>()...)))
{
    return (std::forward<U>(u).*function)(std::forward<Args>(args)...);
}

template <typename Function, typename T, typename U, typename ...Args,
          typename = std::enable_if_t<std::is_base_of<T, std::decay_t<U>>::value>>
decltype(auto) invoke_impl(Function T::* function, U *u, Args &&...args)
    noexcept(noexcept((std::declval<U*>->*function)(std::declval<Args>()...)))
{
    return (u->*function)(std::forward<Args>(args)...);
}

template <typename Function, typename ...Args>
decltype(auto) invoke_impl(Function &&f, Args &&...args)
    noexcept(noexcept(std::declval<Function>()(std::declval<Args>()...)))
{
    return std::forward<Function>(f)(std::forward<Args>(args)...);
}

template <typename Function, typename ...Args>
decltype(auto) invoke(Function &&function, Args &&...args)
    noexcept(noexcept(invoke_impl(std::declval<Function>(),
                                  std::declval<Args>()...)))
{
    return invoke_impl(std::forward<Function>(function), std::forward<Args>(args)...);
}

template <typename Function, typename ...Args>
struct InvokeResult {
    using type = decltype((invoke(std::declval<Function>(),
                                  std::declval<Args>()...)));
};

template <typename Function, typename ...Args>
using InvokeResultT = typename InvokeResult<Function, Args...>::type;

} // namespace detail
} // namespace umigv

#endif

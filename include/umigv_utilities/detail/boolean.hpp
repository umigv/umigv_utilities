#ifndef DETAIL_BOOLEAN_HPP
#define DETAIL_BOOLEAN_HPP

#include "../types.hpp" // umigv::usize

#include <utility> // std::forward

namespace umigv {
namespace detail {

template <typename T, typename U>
bool equal(T &&t, U &&u) noexcept(noexcept(std::forward<T>(t) == std::forward<U>(u))) {
    return std::forward<T>(t) == std::forward<U>(u);
}

template <typename T, typename U>
bool inequal(T &&t, U &&u) noexcept(noexcept(std::forward<T>(t) != std::forward<U>(u))) {
    return std::forward<T>(t) != std::forward<U>(u);
}

constexpr bool boolean_and(bool first) noexcept {
    return first;
}

template <typename ...Types>
constexpr bool boolean_and(bool first, bool second, Types &&...types) noexcept {
    return first and boolean_and(second, std::forward<Types>(types)...);
}

constexpr bool boolean_or(bool first) noexcept {
    return first;
}

template <typename ...Types>
constexpr bool boolean_or(bool first, bool second, Types &&...types) noexcept {
    return first or boolean_or(second, std::forward<Types>(types)...);
}

} // namespace detail
} // namespace umigv

#endif

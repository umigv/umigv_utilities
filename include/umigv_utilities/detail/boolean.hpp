#ifndef DETAIL_BOOLEAN_HPP
#define DETAIL_BOOLEAN_HPP

#include "../types.hpp" // umigv::usize

#include <utility> // std::forward, std::declval

namespace umigv {
namespace detail {

constexpr bool boolean_and(bool first) noexcept {
    return first;
}

template <typename ...Types>
constexpr bool boolean_and(bool first, bool second, Types &&...types)
    noexcept(noexcept(boolean_and(std::declval<bool>(), std::declval<Types>())))
{
    return first and boolean_and(second, std::forward<Types>(types)...);
}

constexpr bool boolean_or(bool first) noexcept {
    return first;
}

template <typename ...Types>
constexpr bool boolean_or(bool first, bool second, Types &&...types)
    noexcept(noexcept(boolean_or(std::declval<bool>(), std::declval<Types>())))
{
    return first or boolean_or(second, std::forward<Types>(types)...);
}

} // namespace detail
} // namespace umigv

#endif

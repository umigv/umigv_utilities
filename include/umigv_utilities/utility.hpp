#ifndef UMIGV_UTILITY_HPP
#define UMIGV_UTILITY_HPP

// utility header; useful but otherwise ungrouped functions

#include "umigv_utilities/ros.hpp"

#include <iterator>
#include <type_traits>

namespace umigv {

// converts to To from From via type punning
template <typename To, typename From,
          typename = std::enable_if_t<sizeof(To) == sizeof(From) &&
                                      alignof(To) == alignof(From)>>
constexpr To& byte_cast(From &from) noexcept {
    return *reinterpret_cast<To*>(&from);
}

// converts to To from From via type punning
template <typename To, typename From,
          typename = std::enable_if_t<sizeof(To) == sizeof(From) &&
                                      alignof(To) == alignof(From)>>
constexpr const To& byte_cast(const From &from) noexcept {
    return *reinterpret_cast<const To*>(&from);
}

template <typename T>
constexpr decltype(auto) adl_begin(T &&t) {
    using std::begin;

    return begin(std::forward<T>(t));
}

template <typename T>
constexpr decltype(auto) adl_end(T &&t) {
    using std::end;

    return end(std::forward<T>(t));
}

} // namespace umigv

#endif

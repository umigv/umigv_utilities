#ifndef UMIGV_UTILITY_HPP
#define UMIGV_UTILITY_HPP

// utility header; whatever useful but otherwise ungrouped functions we need

#include <ros/ros.h> // ros::shutdown, ros::waitForShutdown

#include <cstdlib> // std::exit, EXIT_FAILURE
#include <type_traits> // std::enable_if_t

namespace umigv {

// converts from ToT to FromT via type punning
template <typename ToT, typename FromT,
          typename = std::enable_if_t<sizeof(ToT) == sizeof(FromT) and
                                      alignof(ToT) == alignof(FromT)>>
constexpr ToT& byte_cast(FromT &from) noexcept {
    return *reinterpret_cast<ToT*>(&from);
}

// converts from ToT to FromT via type punning
template <typename ToT, typename FromT,
          typename = std::enable_if_t<sizeof(ToT) == sizeof(FromT) and
                                      alignof(ToT) == alignof(FromT)>>
constexpr const ToT& byte_cast(const FromT &from) noexcept {
    return *reinterpret_cast<const ToT*>(&from);
}

void blocking_shutdown() noexcept __attribute__((noreturn)) {
    ros::shutdown();
    ros::waitForShutdown();
    std::exit(EXIT_FAILURE);
}

} // namespace umigv

#endif

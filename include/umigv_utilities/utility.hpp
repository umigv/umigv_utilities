#ifndef UMIGV_UTILITY_HPP
#define UMIGV_UTILITY_HPP

// utility header; useful but otherwise ungrouped functions

#include <ros/ros.h> // ros::shutdown, ros::waitForShutdown

#include <cstdlib> // std::exit, EXIT_FAILURE
#include <type_traits> // std::enable_if_t

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

void blocking_shutdown() noexcept __attribute__((noreturn));

void blocking_shutdown() noexcept {
    ros::shutdown();
    ros::waitForShutdown();
    std::exit(EXIT_FAILURE);
}

} // namespace umigv

#endif

#ifndef UMIGV_ROS_HPP
#define UMIGV_ROS_HPP

// utilities relating to ROS

#include "umigv_utilities/exceptions.hpp"
#include "umigv_utilities/traits.hpp"

#include <cstdlib>
#include <string>
#include <type_traits>
#include <utility>

#include <boost/optional.hpp>

#include <ros/ros.h>

namespace umigv {

// fetches parameter from rosparam using handle
// throws umigv::ParameterNotFoundException if
// unable to fetch from parameter server
template <typename T, std::enable_if_t<is_rosparam_v<T>, int> = 0>
T get_parameter_fatal(const ros::NodeHandle &handle,
                      const std::string &parameter) {
    T fetched;

    if (not handle.getParam(parameter, fetched)) {
        throw ParameterNotFoundException{ "get_parameter_fatal", parameter };
    }

    return fetched;
}

// fetches parameter from rosparam using handle
// returns default value if unable to fetch
template <typename T, std::enable_if_t<is_rosparam_v<T>, int> = 0>
T get_parameter_or(const ros::NodeHandle &handle, const std::string &name,
                   const T &fallback = T{ }) {
    T fetched;

    handle.param(name, fetched, fallback);

    return fetched;
}

[[noreturn]] inline void blocking_shutdown() noexcept {
    ros::shutdown();
    ros::waitForShutdown();
    std::exit(EXIT_FAILURE);
}

class ParameterServer {
public:
    template <
        typename ...Ts,
        std::enable_if_t<std::is_constructible<ros::NodeHandle, Ts...>::value,
                         int> = 0
    >
    ParameterServer(const bool should_cache, Ts &&...args)
    noexcept(std::is_nothrow_constructible<ros::NodeHandle, Ts...>::value)
    : node_(std::forward<Ts>(args)...), should_cache_{ should_cache } { }

    constexpr void enable_caching() noexcept {
        should_cache_ = true;
    }

    constexpr void disable_caching() noexcept {
        should_cache_ = false;
    }

    bool has_parameter(const std::string &key) const {
        return node_.hasParam(key);
    }

    boost::optional<std::vector<std::string>> get_names() const {
        std::vector<std::string> keys;

        if (!node_.getParamNames(keys)) {
            return boost::none;
        }

        return { std::move(keys) };
    }

    template <typename T, std::enable_if_t<is_rosparam_v<T>, int> = 0>
    boost::optional<T> get(const std::string &key) const {
        T parameter;

        if (!has_parameter(key)) {
            return boost::none;
        }

        if (should_cache_) {
            node_.getParamCached(key, parameter);
        } else {
            node_.getParam(key, parameter);
        }

        return { std::move(parameter) };
    }

private:
    ros::NodeHandle node_;
    bool should_cache_ = false;
};

} // namespace umigv

#endif

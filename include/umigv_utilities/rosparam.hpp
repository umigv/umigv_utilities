#ifndef UMIGV_ROSPARAM_HPP
#define UMIGV_ROSPARAM_HPP

// functions relating to usage of the ROS parameter server

#include "traits.hpp" // umigv::IsParameterV

#include <ros/node_handle.h> // ros::NodeHandle

#include <stdexcept> // std::runtime_error
#include <string> // std::string
#include <type_traits> // std::enable_if_t
#include <utility> // std::move

namespace umigv {

// fetches parameter from rosparam using handle, throws std::runtime_error
// if unable to fetch from parameter server
template <typename T>
std::enable_if_t<IsParameterV<T>, T>
get_parameter_fatal(const ros::NodeHandle &handle,
                    const std::string &parameter) {
    T fetched;

    if (not handle.getParam(parameter, fetched)) {
        throw std::runtime_error{ "get_parameter_fatal" };
    }

    return fetched;
}


template <typename T>
std::enable_if_t<IsParameterV<T>, T>
get_parameter_or(const ros::NodeHandle &handle, const std::string &name,
                 const T &fallback = T{ }) {
    T fetched;

    handle.param(name, fetched, fallback);

    return fetched;
}

} // namespace umigv

#endif

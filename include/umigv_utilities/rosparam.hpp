#ifndef UMIGV_ROSPARAM_HPP
#define UMIGV_ROSPARAM_HPP

// functions relating to usage of the ROS parameter server

#include "exceptions.hpp" // umigv::ParameterNotFoundException
#include "traits.hpp" // umigv::IsParameterV

#include <ros/node_handle.h> // ros::NodeHandle

#include <string> // std::string
#include <type_traits> // std::enable_if_t

namespace umigv {

// fetches parameter from rosparam using handle
// throws umigv::ParameterNotFoundException if
// unable to fetch from parameter server
template <typename T, typename = std::enable_if_t<IsParameterV<T>>>
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
template <typename T, typename = std::enable_if_t<IsParameterV<T>>>
T get_parameter_or(const ros::NodeHandle &handle, const std::string &name,
                   const T &fallback = T{ }) {
    T fetched;

    handle.param(name, fetched, fallback);

    return fetched;
}

} // namespace umigv

#endif

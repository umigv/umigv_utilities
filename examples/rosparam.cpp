#include "umigv_utilities/ros.hpp"
#include "umigv_utilities/types.hpp"

#include <string>
#include <vector>

#include <ros/ros.h>

using namespace umigv::types;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "umigv_utilities_rosparam_test");
    ros::NodeHandle node;
    umigv::ParameterServer params(false);

    const auto val = params.get<f64>("val").value_or(0.0);
    const std::string name = params.get<std::string>("name").value_or("bob");

    params.enable_caching();

    const auto names = params.get_names().value();
    const auto covariance = params.get<std::vector<f64>>("covariance").value();

    const auto rate = params["rate"].value<f64>();
    const auto frequency = params["frequency"].value_or(100.0);
}

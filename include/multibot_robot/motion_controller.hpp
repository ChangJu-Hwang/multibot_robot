#pragma once

#include <geometry_msgs/msg/twist.hpp>

#include "multibot_util/MAPF_Util.hpp"

using namespace MAPF_Util;

namespace Motion
{
    double VelocityComputer(
            const double _max_s, const double _max_vel, const double _max_acc,
            const double _time);

    double TotalMoveTimeComputer(
            const double _max_s, const double _max_vel, const double _max_acc);

} // namespace Motion
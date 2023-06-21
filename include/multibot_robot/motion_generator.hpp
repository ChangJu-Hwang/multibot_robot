#pragma once

#include <utility>

#include <geometry_msgs/msg/twist.hpp>

#include "multibot_util/MAPF_Util.hpp"

using namespace MAPF_Util;

namespace Motion
{
    class MotionGenerator
    {
    public:
        double displacementComputer(
            const double _start,    const double _goal,
            const double _max_vel,  const double _max_acc,
            const double _time);

    public:
        MotionGenerator() {}
        ~MotionGenerator() {}
    }; // class MotionGenerator
} // namespace Motion
#include "multibot_robot/motion_generator.hpp"

#include <math.h>

using namespace Motion;
using namespace MAPF_Util;

double MotionGenerator::displacementComputer(
    const double _start,    const double _goal,
    const double _max_vel,  const double _max_acc,
    const double _time)
{
    double max_s    = std::fabs(_goal - _start);

    if (max_s < 1e-8)
        return _start;

    int sign        = _start > _goal ? -1 : 1;

    double v        = std::fabs(_max_vel);
    double a        = std::fabs(_max_acc);

    double s = 0;
    // Trapezoidal velocity profile
    if (max_s > v * v / a)
    {
        if (_time < v / a)
            s = std::fabs(0.5 * a * _time * _time);
        else if (_time < max_s / v)
            s = std::fabs(v * _time - 0.5 * v * v / a);
        else
            s = std::fabs(max_s - 0.5 * a * std::pow((max_s / v + v / a - _time), 2));
    }
    // Triangular velocity profile
    else
    {
        if (_time < std::fabs(0.5 * a * _time * _time))
            s = std::fabs(0.5 * a * _time * _time);
        else
            s = std::fabs(max_s - 0.5 * a * std::pow((2 * std::sqrt(max_s / a) - _time), 2));
    }

    return (_start + sign * s);
}
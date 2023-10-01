#include "multibot_robot/motion.hpp"

#include <math.h>

using namespace MAPF_Util;

double Motion::DisplacementComputer(
    const double _max_s, const double _max_vel, const double _max_acc,
    const double _time)
{
    if (std::fabs(_max_s) < 1e-8)
        return 0.0;
    
    // Trapezoidal Velocity Profile
    if (std::fabs(_max_s) > _max_vel * _max_vel / _max_acc + 1e-8)
    {
        if (_time < _max_vel / _max_acc)
            return (0.5 * _max_acc * _time * _time);
        else if (_time < std::fabs(_max_s) / _max_vel)
            return (_max_vel * _time - 0.5 * _max_vel * _max_vel / _max_acc);
        else
            return (std::fabs(_max_s) - 0.5 * _max_acc * std::pow(std::fabs(_max_s) / _max_vel + _max_vel / _max_acc - _time,2));
    }
    // Triangular Velocity Profile
    else
    {
        if (_time < std::sqrt(std::fabs(_max_s) / _max_acc))
            return (0.5 * _max_acc * _time * _time);
        else
            return (std::fabs(_max_s) - 0.5 * _max_acc * std::pow(2 * std::sqrt(std::fabs(_max_s) / _max_acc) - _time,2));
    }
}

double Motion::VelocityComputer(
    const double _max_s, const double _max_vel, const double _max_acc,
    const double _time)
{
    double v = 0;

    if (std::fabs(_max_s) < 1e-8)
        return v;
    
    if (TotalMoveTimeComputer(_max_s, _max_vel, _max_acc) < _time)
        return v;

    double max_v    = std::fabs(_max_vel);
    double max_a    = std::fabs(_max_acc);

    // Trapezoidal velocity profile
    if (std::fabs(_max_s) > max_v * max_v / max_a)
    {
        if (_time < max_v / max_a)
            v = max_a * _time;
        else if (_time < std::fabs(_max_s) / max_v)
            v = max_v;
        else
            v = -1 * max_a * (_time - std::fabs(_max_s) / max_v) + max_v;
    }
    // Triangular velocity profile
    else
    {
        if (_time < max_v / max_a)
            v = max_a * _time;
        else
            v = -1 * max_a * (_time - max_v / max_a) + max_v;
    }

    return v;
}

double Motion::TotalMoveTimeComputer(
    const double _max_s, const double _max_vel, const double _max_acc)
{
    double totalMoveTime;

    // Trapezoidal velocity profile
    if (std::fabs(_max_s) > _max_vel * _max_vel / _max_acc)
        totalMoveTime = std::fabs(_max_s) / _max_vel + _max_vel / _max_acc;
    // Triangular velocity profile
    else
        totalMoveTime = 2 * std::sqrt(std::fabs(_max_s) / _max_acc);

    return totalMoveTime;
}
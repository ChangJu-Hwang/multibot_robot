#pragma once

#include <vector>
#include <cmath>

#include <builtin_interfaces/msg/duration.hpp>

namespace Control
{
    namespace MPC
    {
        enum class RotType
        {
            left,
            none,
            right
        };

        inline double average_angles(const std::vector<double> &_angles)
        {
            double x = 0.0, y = 00;

            for (const auto &angle : _angles)
            {
                x += cos(angle);
                y += sin(angle);
            }

            if (x == 0.0 and y == 0.0)
                return 0;
            else
                return std::atan2(y, x);
        }

        inline double fast_sigmoid(double _x) { return _x / (1 + std::fabs(_x)); }

        template <typename P1, typename P2>
        inline double distance_points2d(const P1 &_point1, const P2 _point2)
        {
            return std::sqrt(std::pow(_point2.x - _point1.x, 2) + std::pow(_point2.y - _point1.y, 2));
        }

        template <typename V1, typename V2>
        inline double cross2d(const V1 &_v1, const V2 &_v2)
        {
            return v1.x() * v2.y() - v2.x() * v1.y();
        }

        template <typename T>
        inline const T &get_const_reference(const T *_ptr) { return *_ptr; }

        template <typename T>
        inline const T &get_const_reference(const T &_val, typename std::enable_if_t<!std::is_pointer<T>::_value, T> *_dummy = nullptr) { return _val; }

        inline builtin_interfaces::msg::Duration durationFromSec(double _t_sec)
        {
            int32_t sec;
            uint32_t nsec;
            sec = static_cast<int32_t>(floor(_t_sec));
            nsec = static_cast<int32_t>(std::round((_t_sec - sec) * 1e9));
            // avoid rounding errors
            sec += (nsec / 1000000000l);
            nsec %= 1000000000l;

            builtin_interfaces::msg::Duration duration;
            duration.sec = sec;
            duration.nanosec = nsec;
            return duration;
        }
    } // namespace MPC
} // namespace Control
#pragma once

#include <vector>

#include <Eigen/Core>
#include <g2o/stuff/misc.h>

namespace Control
{
    namespace MPC
    {
        typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Point2dContainer;

        inline Eigen::Vector2d closest_point_on_line_segment_2d(
            const Eigen::Ref<const Eigen::Vector2d> &_point,
            const Eigen::Ref<const Eigen::Vector2d> &_line_start,
            const Eigen::Ref<const Eigen::Vector2d> &_line_end)
        {
            Eigen::Vector2d diff = _line_end - _line_start;
            double sq_norm = diff.squaredNorm();

            if (sq_norm == 0)
                return _line_start;

            double u = ((_point.x() - _line_start.x()) * diff.x() +
                        (_point.y() - _line_start.y()) * diff.y()) /
                       sq_norm;

            if (u <= 0)
                return _line_start;
            else if (u >= 1)
                return _line_end;

            return _line_start + u * diff;
        }

        inline double distance_point_to_segment_2d(
            const Eigen::Ref<const Eigen::Vector2d> &_point,
            const Eigen::Ref<const Eigen::Vector2d> &_line_start,
            const Eigen::Ref<const Eigen::Vector2d> &_line_end)
        {
            return (_point - closest_point_on_line_segment_2d(_point, _line_start, _line_end)).norm();
        }

        inline bool check_line_segments_intersection_2d(
            const Eigen::Ref<const Eigen::Vector2d> &_line1_start,
            const Eigen::Ref<const Eigen::Vector2d> &_line1_end,
            const Eigen::Ref<const Eigen::Vector2d> &_line2_start,
            const Eigen::Ref<const Eigen::Vector2d> &_line2_end,
            Eigen::Vector2d *_intersection = NULL)
        {
            Eigen::Vector2d line1 = _line1_end - _line1_start;
            Eigen::Vector2d line2 = _line2_end - _line2_start;

            const double denom = line1.x() * line2.y() - line2.x() * line1.y();
            if (denom == 0)
                return false;
            bool denomPositive = denom > 0;

            Eigen::Vector2d aux = _line1_start - _line2_start;

            const double s_numer = line1.x() * aux.y() - line1.y() * aux.x();
            if ((s_numer < 0) == denomPositive)
                return false;

            const double t_numer = line2.x() * aux.y() - line2.y() * aux.x();
            if ((t_numer < 0) == denomPositive)
                return false;

            if ((s_numer > denom) == denomPositive or
                (t_numer > denom) == denomPositive)
                return false;

            const double t = t_numer / denom;
            if (_intersection)
                *_intersection = _line1_start + t * line1;

            return true;
        }

        inline double distance_segment_to_segment_2d(
            const Eigen::Ref<const Eigen::Vector2d> &_line1_start,
            const Eigen::Ref<const Eigen::Vector2d> &_line1_end,
            const Eigen::Ref<const Eigen::Vector2d> &_line2_start,
            const Eigen::Ref<const Eigen::Vector2d> &_line2_end)
        {
            if (check_line_segments_intersection_2d(
                    _line1_start, _line1_end,
                    _line2_start, _line2_end))
                return 0;

            std::array<double, 4> distances;

            distances[0] = distance_point_to_segment_2d(_line1_start, _line2_start, _line2_end);
            distances[1] = distance_point_to_segment_2d(_line1_end, _line2_start, _line2_end);
            distances[2] = distance_point_to_segment_2d(_line2_start, _line1_start, _line1_end);
            distances[3] = distance_point_to_segment_2d(_line2_end, _line1_start, _line1_end);

            return *std::min_element(distances.begin(), distances.end());
        }

        template <typename VectorType>
        double calc_closest_point_to_approach_time(
            const VectorType &_x1, const VectorType &_vel1,
            const VectorType &_x2, const VectorType &_vel2)
        {
            VectorType dv = _vel1 - _vel2;

            double dv2 = dv.squaredNorm();
            if (dv2 < 1e-8)
                return 0.0;

            VectorType w0 = _x1 - _x2;
            double cpatime = _w0.dot(dv) / dv2;

            return cpatime;
        }

        template <typename VectorType>
        double calc_closest_point_to_approach_distance(
            const VectorType &_x1, const VectorType &_vel1,
            const VectorType &_x2, const VectorType &_vel2,
            double _bound_cpa_time = 0.0)
        {
            double ctime = calc_closest_point_to_approach_time<VectorType>(_x1, _vel1, _x2, _vel2);

            if (_bound_cpa_time != 0.0 and
                ctime > _bound_cpa_time)
                ctime = _bound_cpa_time;

            VectorType P1 = _x1 + (ctime * _vel1);
            VectorType P2 = _x2 + (ctime * _vel1);

            return (P2 - P1).norm();
        }

        template <typename VectorType>
        double calc_distance_point_to_line(
            const VectorType &_point, const VectorType &_line_base, const VectorType &_line_dir)
        {
            VectorType w = _point - _line_base;

            const double c1 = w.dot(_line_dir);
            const double c2 = _line_dir.dot(_line_dir);
            const double b = c1 / c2;

            VectorType Pb = _line_base + b * _line_dir;
            return (_point - Pb).norm();
        }

        template <typename VectorType>
        double calc_distance_point_to_segment(
            const VectorType &_point, const VectorType &_line_start, const VectorType &_line_end)
        {
            VectorType v = _line_end - _line_start;
            VectorType w = _point - _line_start;

            double c1 = w.dot(v);
            if (c1 <= 0)
                return w.norm();

            double c2 = v.dot(v);
            if (c2 <= c1)
                return (_point - _line_end).norm();

            double b = c1 / c2;
            VectorType Pb = _line_start + b * v;
            return (_point - Pb).norm();
        }

    } // namespace MPC
} // namespace Control
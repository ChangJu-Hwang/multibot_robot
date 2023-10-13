#pragma once

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <g2o/core/base_vertex.h>

#include "multibot_robot/g2o_utils/pose_se2.hpp"

namespace Control
{
    namespace MPC
    {
        class VertexPose : public g2o::BaseVertex<5, PoseSE2>
        {
        public:
            VertexPose(bool _fixed = false)
            {
                setToOriginImpl();
                setFixed(_fixed);
            }

            VertexPose(const PoseSE2 &_state, bool _fixed = false)
            {
                _estimate = _state;
                setFixed(_fixed);
            }

            VertexPose(
                const Eigen::Ref<const Eigen::Vector2d> &_position,
                double _theta,
                bool _fixed = false)
            {
                _estimate.position() = _position;
                _estimate.theta() = _theta;
                setFixed(_fixed);
            }

            VertexPose(
                const trajectory_msgs::msg::JointTrajectoryPoint &_trajectoryPoint,
                bool _fixed = false)
            {
                _estimate.x() = _trajectoryPoint.positions[0];
                _estimate.y() = _trajectoryPoint.positions[1];
                _estimate.theta() = _trajectoryPoint.positions[2];
                setFixed(_fixed);
            }

            VertexPose(
                double _x, double _y, double _theta,
                bool _fixed = false)
            {
                _estimate.x() = _x;
                _estimate.y() = _y;
                _estimate.theta() = _theta;
                setFixed(_fixed);
            }

            ~VertexPose() {}

        public:
            PoseSE2 &pose() { return _estimate; }
            const PoseSE2 &pose() const { return _estimate; }

            Eigen::Vector2d &position() { return _estimate.position(); }
            const Eigen::Vector2d &position() const { return _estimate.position(); }

            double &x() { return _estimate.x(); }
            const double &x() const { return _estimate.x(); }

            double &y() { return _estimate.y(); }
            const double &y() const { return _estimate.y(); }

            double &theta() { return _estimate.theta(); }
            const double &theta() const { return _estimate.theta(); }

        public:
            virtual void setToOriginImpl()
            {
                _estimate.setZero();
            }

            virtual void oplusImpl(const double *update)
            {
                _estimate.plus(update);
            }

            virtual bool read(std::istream &_is)
            {
                _is >> _estimate.x() >> _estimate.y() >> _estimate.theta();
                return true;
            }

            virtual bool write(std::ostream &_os) const
            {
                _os << _estimate.x() << " "
                    << _estimate.y() << " "
                    << _estimate.theta() << " ";
                return _os.good();
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class VertexPose
    }      // namespace MPC
} // namespace Control
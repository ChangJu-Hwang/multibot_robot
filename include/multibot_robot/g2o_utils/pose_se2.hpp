#pragma once

#include <Eigen/Core>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <g2o/stuff/misc.h>

namespace Control
{
    namespace MPC
    {
        class PoseSE2
        {
        public:
            PoseSE2()
            {
                setZero();
            }

            PoseSE2(
                const Eigen::Ref<const Eigen::Vector2d> &_position, double _theta)
            {
                position_ = _position;
                theta_ = _theta;
            }

            PoseSE2(const trajectory_msgs::msg::JointTrajectoryPoint &_trajectoryPoint)
            {
                position_.coeffRef(0) = _trajectoryPoint.positions[0];
                position_.coeffRef(1) = _trajectoryPoint.positions[1];
                position_.coeffRef(2) = _trajectoryPoint.positions[2];
            }

            PoseSE2(
                double _x, double _y, double _theta)
            {
                position_.coeffRef(0) = _x;
                position_.coeffRef(1) = _y;
                theta_ = _theta;
            }

            PoseSE2(
                const geometry_msgs::msg::Pose &_pose)
            {
                position_.coeffRef(0) = _pose.position.x;
                position_.coeffRef(1) = _pose.position.y;
                theta_ = tf2::getYaw(_pose.orientation);
            }

            PoseSE2(
                const geometry_msgs::msg::PoseStamped &_pose)
                : PoseSE2(_pose.pose) {}

            PoseSE2(
                const geometry_msgs::msg::Pose2D &_pose)
            {
                position_.coeffRef(0) = _pose.x;
                position_.coeffRef(1) = _pose.y;
                theta_ = _pose.theta;
            }

            PoseSE2(const PoseSE2 &_pose)
            {
                position_ = _pose.position_;
                theta_ = _pose.theta_;
            }

            ~PoseSE2() {}

        public:
            Eigen::Vector2d &position() { return position_; }
            const Eigen::Vector2d &position() const { return position_; }

            double &x() { return position_.coeffRef(0); }
            const double &x() const { return position_.coeffRef(0); }

            double &y() { return position_.coeffRef(1); }
            const double &y() const { return position_.coeffRef(1); }

            double &theta() { return theta_; }
            const double &theta() const { return theta_; }

        public:
            void setZero()
            {
                position_.setZero();
                theta_ = 0.0;
            }

            void toPoseMsg(geometry_msgs::msg::Pose &_pose) const
            {
                _pose.position.x = position_.x();
                _pose.position.y = position_.y();
                _pose.position.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, theta_);
                _pose.orientation = tf2::toMsg(q);
            }

            void toPoseMsg(geometry_msgs::msg::Pose2D &_pose) const
            {
                _pose.x = position_.x();
                _pose.y = position_.y();
                _pose.theta = theta_;
            }

            Eigen::Vector2d orientationUnitVec() const
            {
                return Eigen::Vector2d(std::cos(theta_), std::sin(theta_));
            }

            void scale(double _factor)
            {
                position_ *= _factor;
                theta_ = g2o::normalize_theta(theta_ * _factor);
            }

            void plus(const double *_state_as_array)
            {
                position_.coeffRef(0) += _state_as_array[0];
                position_.coeffRef(1) += _state_as_array[1];
                theta_ = g2o::normalize_theta(theta_ + _state_as_array[2]);
            }

            void averageInPlace(const PoseSE2 &_state1, const PoseSE2 &_state2)
            {
                position_ = (_state1.position_ + _state2.position_) / 2;
                theta_ = g2o::average_angle(_state1.theta_, _state2.theta_);
            }

            static PoseSE2 average(const PoseSE2 &_state1, const PoseSE2 &_state2)
            {
                return PoseSE2(
                    (_state1.position_ + _state2.position_) / 2,
                    g2o::average_angle(_state1.theta_, _state2.theta_));
            }

        public:
            PoseSE2 &operator=(const PoseSE2 &_rhs)
            {
                if (&_rhs != this)
                {
                    position_ = _rhs.position_;
                    theta_ = _rhs.theta_;
                }
                return *this;
            }

            PoseSE2 &operator+=(const PoseSE2 &_rhs)
            {
                position_ += _rhs.position_;
                theta_ += _rhs.theta_;
            }

            friend PoseSE2 operator+(PoseSE2 _lhs, const PoseSE2 &_rhs)
            {
                return _lhs += _rhs;
            }

            PoseSE2 &operator-=(const PoseSE2 &_rhs)
            {
                position_ -= _rhs.position_;
                theta_ -= _rhs.theta_;
            }

            friend PoseSE2 operator-(PoseSE2 _lhs, const PoseSE2 &_rhs)
            {
                return _lhs -= _rhs;
            }

            friend PoseSE2 operator*(PoseSE2 _pose, double _scalar)
            {
                _pose.position_ *= _scalar;
                _pose.theta_ *= _scalar;
                return _pose;
            }

            friend PoseSE2 operator*(double _scalar, PoseSE2 _pose)
            {
                _pose.position_ *= _scalar;
                _pose.theta_ *= _scalar;
                return _pose;
            }

            friend std::ostream &operator<<(std::ostream &_os, const PoseSE2 &_pose)
            {
                _os << "x: " << _pose.position_[0]
                    << " y: " << _pose.position_[1]
                    << " theta: " << _pose.theta_;
                return _os;
            }

        private:
            Eigen::Vector2d position_;
            double theta_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class PoseSE2
    }      // namespace MPC
} // namespace Control
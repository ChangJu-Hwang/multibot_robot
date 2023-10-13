#pragma once

#include <geometry_msgs/msg/twist.hpp>

#include "multibot_robot/g2o_utils/base_mpc_edges.hpp"
#include "multibot_robot/g2o_utils/vertex_pose.hpp"
#include "multibot_robot/g2o_utils/vertex_timediff.hpp"
#include "multibot_robot/g2o_utils/penalties.hpp"
#include "multibot_robot/g2o_utils/g2o_utils.hpp"

namespace Control
{
    namespace MPC
    {
        class EdgeVelocity : public BaseMPCMultiEdge<2, const geometry_msgs::msg::Twist*>
        {
        public:
            EdgeVelocity()
            {
                this->resize(3);
            }

        public:
            void computeError()
            {
                const VertexPose *pose1 = static_cast<const VertexPose *>(_vertices[0]);
                const VertexPose *pose2 = static_cast<const VertexPose *>(_vertices[1]);
                const VertexTimeDiff *deltaT = static_cast<const VertexTimeDiff *>(_vertices[2]);

                const Eigen::Vector2d deltaS = pose2->estimate().position() - pose1->estimate().position();

                const double dist = deltaS.norm();
                const double angle_diff = g2o::normalize_theta(pose2->theta() - pose1->theta());

                double vel = dist / deltaT->estimate();
                vel *= fast_sigmoid(100 * (deltaS.x() * cos(pose1->theta()) + deltaS.y() * sin(pose1->theta())));

                const double omega = angle_diff / deltaT->estimate();

                _error[0] = std::fabs(vel - _measurement->linear.x);
                _error[1] = std::fabs(omega - _measurement->angular.z);

                // _error[0] = penaltyBoundToInterval(vel, max_vel_lin_, 0.0);
                // _error[1] = penaltyBoundToInterval(omega, max_vel_ang_, 0.0);
            }

            void setVelocity(const geometry_msgs::msg::Twist &_vel)
            {
                _measurement = &_vel;
            }

            void setMaxVelocity(double _max_vel_lin, double _max_vel_ang)
            {
                max_vel_lin_ = _max_vel_lin;
                max_vel_ang_ = _max_vel_ang;
            }

        private:
            double max_vel_lin_;
            double max_vel_ang_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class EdgeVelocity
    }      // namespace MPC

} // namespace Control
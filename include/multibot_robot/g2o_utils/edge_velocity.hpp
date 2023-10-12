#pragma once

#include "multibot_robot/g2o_utils/base_mpc_edges.hpp"
#include "multibot_robot/g2o_utils/vertex_pose.hpp"
#include "multibot_robot/g2o_utils/vertex_timediff.hpp"
#include "multibot_robot/g2o_utils/penalties.hpp"
#include "multibot_robot/g2o_utils/g2o_utils.hpp"

namespace Control
{
    namespace MPC
    {
        class EdgeVelocity : public BaseMPCMultiEdge<2, Eigen::Vector2d>
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

                _error[0] = std::fabs(vel - _measurement.coeffRef(0));
                _error[1] = std::fabs(omega - _measurement.coeffRef(1));
            }

            void setVelocity(double _vel_x, double _vel_theta)
            {
                _measurement.coeffRef(0) = _vel_x;
                _measurement.coeffRef(1) = _vel_theta;
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class EdgeVelocity
    }      // namespace MPC

} // namespace Control
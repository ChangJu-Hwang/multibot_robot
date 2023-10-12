#pragma once

#include <cmath>

#include "multibot_robot/g2o_utils/base_mpc_edges.hpp"
#include "multibot_robot/g2o_utils/vertex_pose.hpp"
#include "multibot_robot/g2o_utils/penalties.hpp"

namespace Control
{
    namespace MPC
    {
        class EdgeKinematicsDiffDrive : public BaseMPCBinaryEdge<2, double, VertexPose, VertexPose>
        {
        public:
            EdgeKinematicsDiffDrive()
            {
                this->setMeasurement(0.0);
            }

        public:
            void computeError()
            {
                const VertexPose *pose1 = static_cast<const VertexPose *>(_vertices[0]);
                const VertexPose *pose2 = static_cast<const VertexPose *>(_vertices[1]);

                Eigen::Vector2d deltaS = pose2->position() - pose1->position();

                _error[0] = std::fabs((cos(pose1->theta()) + cos(pose2->theta())) * deltaS[1] - (sin(pose1->theta()) + sin(pose2->theta())) * deltaS[0]);

                Eigen::Vector2d angle_vec (cos(pose1->theta()), sin(pose1->theta()));
                _error[1] = penaltyBoundFromBelow(deltaS.dot(angle_vec), 0.0, 0.0);
            }
        
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class EdgeKinematicsDiffDrive
    }      // namespace MPC
} // namespace Control
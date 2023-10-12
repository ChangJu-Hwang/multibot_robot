#pragma once

#include <g2o/stuff/misc.h>

#include "multibot_robot/g2o_utils/base_mpc_edges.hpp"
#include "multibot_robot/g2o_utils/vertex_pose.hpp"
#include "multibot_robot/g2o_utils/penalties.hpp"

namespace Control
{
    namespace MPC
    {
        class EdgePreferRotDir : public BaseMPCBinaryEdge<1, double, VertexPose, VertexPose>
        {
        public:
            EdgePreferRotDir()
            {
                this->setMeasurement(1.0);
            }

        public:
            void computeError()
            {
                const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
                const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);

                _error[0] = penaltyBoundFromBelow(_measurement * g2o::normalize_theta(pose2->theta() - pose1->theta()), 0.0, 0.0);
            }

            void preferRight()
            {
                this->setMeasurement(-1.0);
            }

            void preferLeft()
            {
                this->setMeasurement(1.0);
            }
        
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class EdgePreferRotDir
    }      // namespace MPC
} // namespace Control
#pragma once

#include "multibot_robot/g2o_utils/base_mpc_edges.hpp"
#include "multibot_robot/g2o_utils/vertex_pose.hpp"

namespace Control
{
    namespace MPC
    {
        class EdgePositionOptimal: public BaseMPCBinaryEdge<1, double, VertexPose, VertexPose>
        {
        public:
            EdgePositionOptimal()
            {
                this->setMeasurement(0.0);
            }
        public:
            void computeError()
            {
                const VertexPose *pose1 = static_cast<const VertexPose*>(_vertices[0]);
                const VertexPose *pose2 = static_cast<const VertexPose*>(_vertices[1]);

                _error[0] = (pose2->position() - pose1->position()).norm();
            }
        
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // EdgePositionOptimal
    } // namespace MPC
} // namespace Control
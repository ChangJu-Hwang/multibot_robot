#pragma once

#include "multibot_robot/g2o_utils/base_mpc_edges.hpp"
#include "multibot_robot/g2o_utils/vertex_pose.hpp"

namespace Control
{
    namespace MPC
    {
        class EdgeWaypoint : public BaseMPCUnaryEdge<1, const Eigen::Vector2d*, VertexPose>
        {
        public:
            EdgeWaypoint()
            {
                _measurement = NULL;
            }
        
        public:
            void computeError()
            {
                const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

                _error[0] = (bandpt->position() - *_measurement).norm();
            }

            void setWaypoint(const Eigen::Vector2d* _way_point)
            {
                _measurement = _way_point;
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    } // namespace MPC
} // namespace Control
#pragma once

#include "multibot_robot/g2o_utils/base_mpc_edges.hpp"
#include "multibot_robot/g2o_utils/vertex_timediff.hpp"

namespace Control
{
    namespace MPC
    {
        class EdgeTimeOptimal : public BaseMPCUnaryEdge<1, double, VertexTimeDiff>
        {
        public:
            EdgeTimeOptimal()
            {
                this->setMeasurement(0.0);
            }
        
        public:
            void computeError()
            {
                const VertexTimeDiff *timeDiff = static_cast<const VertexTimeDiff*>(_vertices[0]);

                _error[0] = timeDiff->dt();
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class EdgeTimeOptimal
    } // namespace MPC
} // namespace Control
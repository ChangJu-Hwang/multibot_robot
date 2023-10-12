#pragma once

#include "g2o/core/base_vertex.h"

namespace Control
{
    namespace MPC
    {
        class VertexTimeDiff : public g2o::BaseVertex<1, double>
        {
        public:
            VertexTimeDiff(bool _fixed = false)
            {
                setToOriginImpl();
                setFixed(_fixed);
            }

            VertexTimeDiff(double _dt, bool _fixed = false)
            {
                _estimate = _dt;
                setFixed(fixed);
            }

            ~VertexTimeDiff() {}

        public:
            double &dt() { return _estimate; }

            const double &dt() const { return _estimate; }

            virtual void setToOriginImpl()
            {
                _estimate = 0.1;
            }

            virtual void oplusImpl(const double *update)
            {
                _estimate += *update;
            }

            virtual bool read(std::istream &_is)
            {
                _is >> _estimate;
                return true;
            }

            virtual bool write(std::ostream &_os) const
            {
                _os << estimate();
                return _os.good();
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class VertexTimeDiff
    }
}
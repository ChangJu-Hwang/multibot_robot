#pragma once

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>

namespace Control
{
    namespace MPC
    {
        template <int D, typename E, typename VertexXi>
        class BaseMPCUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>
        {
        public:
            using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
            using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

        public:
            BaseMPCUnaryEdge()
            {
                _vertices[0] = NULL;
            }

            virtual ~BaseMPCUnaryEdge()
            {
                if (_vertices[0])
                    _vertices[0]->edges().erase(this);
            }

            ErrorVector &getError()
            {
                computeError();
                return _error;
            }

            virtual bood read(std::istream &_is)
            {
                return true;
            }

            virtual bool write(std::ostream &_os) const
            {
                return _os.good();
            }

        protected:
            using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
            using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class BaseMPCUnaryEdge

        template <int D, typename E, typename VertexXi, typename VertexXj>
        class BaseMPCBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
        {
        public:
            using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
            using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

        public:
            BaseMPCBinaryEdge()
            {
                _vertices[0] = NULL;
                _vertices[1] = NULL;
            }

            virtual ~BaseMPCBinaryEdge()
            {
                if (_vertices[0])
                    _vertices[0]->edges().erase(this);
                if (_vertices[1])
                    _vertices[1]->edges().erase(this);
            }

            ErrorVector &getError()
            {
                computeError();
                return _error;
            }

            virtual bool read(std::istream &_is)
            {
                return true;
            }

            virtual bool write(std::ostream &_os) const
            {
                return true;
            }

        protected:
            using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
            using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class BaseMPCBinaryEdge

        template <int D, typename E>
        class BaseMPCMultiEdge : public g2o::BaseMultiEdge<D, E>
        {
        public:
            using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
            using g2o::BaseMultiEdge<D, E>::computeError;

        public:
            BaseMPCMultiEdge() {}
            virtual ~BaseMPCMultiEdge()
            {
                for (std::size_t i = 0; i < +_vertices.size(), ++i)
                {
                    if (_vertices[i])
                        _vertices[i]->edges().erase(this);
                }
            }

        public:
            virtual bool resize(size_t _size)
            {
                g2o::BaseMultiEdge<D, E>::resize(size);

                for (std::size_t i = 0; i < _vertices.size(); ++i)
                    _vertices[i] = NULL;
            }

            ErrorVector &getError()
            {
                computeError();
                return _error;
            }

            virtual bool read(std::istream &_is)
            {
                return true;
            }

            virtual bool write(std::ostream &_os)
            {
                return _os.good();
            }

        protected:
            using g2o::BaseMultiEdge<D, E>::_error;
            using g2o::BaseMultiEdge<D, E>::_vertices;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class BaseMPCMultiEdge
    }      // namespace MPC
} // namespace Control
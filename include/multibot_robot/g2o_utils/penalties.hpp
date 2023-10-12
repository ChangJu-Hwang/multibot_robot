#pragma once;

#include <cmath>
#include <Eigen/Core>
#include <g2o/stuff/misc.h>

namespace Control
{
  namespace MPC
  {
    inline double penaltyBoundToInterval(const double &var, const double &a, const double &epsilon)
    {
      if (var < -a + epsilon)
      {
        return (-var - (a - epsilon));
      }
      if (var <= a - epsilon)
      {
        return 0.;
      }
      else
      {
        return (var - (a - epsilon));
      }
    }

    inline double penaltyBoundToInterval(const double &var, const double &a, const double &b, const double &epsilon)
    {
      if (var < a + epsilon)
      {
        return (-var + (a + epsilon));
      }
      if (var <= b - epsilon)
      {
        return 0.;
      }
      else
      {
        return (var - (b - epsilon));
      }
    }

    inline double penaltyBoundFromBelow(const double &var, const double &a, const double &epsilon)
    {
      if (var >= a + epsilon)
      {
        return 0.;
      }
      else
      {
        return (-var + (a + epsilon));
      }
    }

    inline double penaltyBoundToIntervalDerivative(const double &var, const double &a, const double &epsilon)
    {
      if (var < -a + epsilon)
      {
        return -1;
      }
      if (var <= a - epsilon)
      {
        return 0.;
      }
      else
      {
        return 1;
      }
    }

    inline double penaltyBoundToIntervalDerivative(const double &var, const double &a, const double &b, const double &epsilon)
    {
      if (var < a + epsilon)
      {
        return -1;
      }
      if (var <= b - epsilon)
      {
        return 0.;
      }
      else
      {
        return 1;
      }
    }

    inline double penaltyBoundFromBelowDerivative(const double &var, const double &a, const double &epsilon)
    {
      if (var >= a + epsilon)
      {
        return 0.;
      }
      else
      {
        return -1;
      }
    }
  } // namespace MPC
} // namespace Control

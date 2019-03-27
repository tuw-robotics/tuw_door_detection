
//
// Created by felix on 22.03.19.
//

#ifndef TUW_SENSOR_MODEL_PARAMETER_ESTIMATOR_EM_H
#define TUW_SENSOR_MODEL_PARAMETER_ESTIMATOR_EM_H

#include <map>
#include <vector>
#include <cmath>
#include <limits>

namespace tuw
{
  
  class SensorModelParameterEstimatorEM
  {
  
  public:
    
    struct ParametersEstimated
    {
      double z_hit;
      double z_max;
      double z_rand;
      double z_short;
      double sigma_hit;
      double lambda_short;
      
      inline double dotMinus(ParametersEstimated &other)
      {
        double x = other.z_hit - z_hit;
        double y = other.z_max - z_max;
        double z = other.z_rand - z_rand;
        double w = other.z_short - z_short;
        double u = other.sigma_hit - sigma_hit;
        double v = other.lambda_short - lambda_short;
        return std::sqrt(x*x - y*y - z*z - w*w - u*u - v*v);
      }
    };
    
    void add( double measured, double expected );
    
    void clear();
    
    ParametersEstimated &compute();
  
  private:
    ParametersEstimated params_;
    
    inline double pHit( double z, double z_k_star, double sigma_hit )
    {
      if ( params_.z_max >= z && z >= 0 )
      {
        double sample = z - z_k_star;
        double sigma_hit_squared = std::pow( params_.sigma_hit, 2 );
        return 1.0 / sqrt( 2 * M_PI * sigma_hit_squared ) * exp( -0.5 * (std::pow( sample, 2 ) / sigma_hit_squared));
      }
      return 0;
    }
    
    inline double pZmax( double z, double z_max )
    {
      if ((z - z_max) < std::numeric_limits<float>::epsilon())
      {
        return 1;
      }
      return 0;
    }
    
    inline double pShort( double z, double z_k_star, double lambda_short )
    {
      if (0 <= z && z <= z_k_star )
      {
        return (1.0 / (1.0 - exp(-lambda_short*z_k_star))) * lambda_short * exp(- lambda_short * z);
      }
      return 0;
    }
    
    inline double pRand( double z, double z_max )
    {
      if (0 <= z && z <= z_max)
      {
        return 1.0 / z_max;
      }
    }
    
    std::vector<std::pair<double, double>> z_m_e_;
  };
  
};
#endif //PROJECT_SENSOR_MODEL_PARAMETER_ESTIMATOR_EM_H

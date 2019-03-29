
//
// Created by felix on 22.03.19.
//

#ifndef TUW_SENSOR_MODEL_PARAMETER_ESTIMATOR_EM_H
#define TUW_SENSOR_MODEL_PARAMETER_ESTIMATOR_EM_H

#include <map>
#include <vector>
#include <cmath>
#include <limits>
#include <memory>

namespace tuw
{
  class SensorModelParameterEstimatorEM
  {
  
  public:
    
    class ParametersEstimated
    {
    public:
      //TODO move z_ stuff to config class
      double z_hit;
      double z_max;
      double z_rand;
      double z_short;
      double sigma_hit;
      double w_hit;
      double w_max;
      double w_rand;
      double w_short;
      double lambda_short;
      
      ParametersEstimated(double _z_hit, double _z_max, double _z_rand, double _z_short, double _sigma_hit, double _lambda_short)
      {
        z_hit = _z_hit;
        z_max = _z_max;
        z_rand = _z_rand;
        z_short = _z_short;
        sigma_hit = _sigma_hit;
        lambda_short = _lambda_short;
        w_hit = 0;
        w_max = 0;
        w_rand = 0;
        w_short = 0;
      }
      
      ParametersEstimated(const ParametersEstimated &other)
      {
        this->z_hit = other.z_hit;
        this->z_max = other.z_max;
        this->z_rand = other.z_rand;
        this->z_short = other.z_short;
        this->sigma_hit = other.sigma_hit;
        this->lambda_short = other.lambda_short;
        this->w_hit = other.w_hit;
        this->w_max = other.w_max;
        this->w_rand = other.w_rand;
        this->w_short = other.w_short;
      }
      
      inline double dotMinus(ParametersEstimated &other)
      {
        double x = other.w_hit - w_hit;
        double y = other.w_max - w_max;
        double z = other.w_rand - w_rand;
        double w = other.w_short - w_short;
        double u = other.sigma_hit - sigma_hit;
        double v = other.lambda_short - lambda_short;
        return std::sqrt(x*x + y*y + z*z + w*w + u*u + v*v);
      }
    };
    
    SensorModelParameterEstimatorEM();
    
    void add( double measured, double expected );
    
    void clear();
  
    void setParams(ParametersEstimated &params);
    
    std::shared_ptr<ParametersEstimated> compute();
  
  private:
    std::shared_ptr<ParametersEstimated> params_;
    
    inline double pHit( double z, double z_k_star, double sigma_hit )
    {
      if ( params_->z_max >= z && z >= 0 )
      {
        double sigma_hit_squared = std::pow( params_->sigma_hit, 2 );
        double normalzer = 1.0 / std::sqrt(sigma_hit_squared * 2  * M_PI) * params_->z_max * exp(-pow(z_k_star, 2.0) / (2.0 * sigma_hit_squared));
        normalzer = 1.0 / normalzer;
        double sample = z - z_k_star;
        return normalzer * 1.0 / sqrt( 2 * M_PI * sigma_hit_squared ) * exp( -0.5 * (std::pow( sample, 2 ) / sigma_hit_squared));
      }
      return 0;
    }
    
    inline double pZmax( double z, double z_max )
    {
      if (fabs(z - z_max) < std::numeric_limits<float>::epsilon())
      {
        return 1.0;
      }
      return 0.0;
    }
    
    inline double pShort( double z, double z_k_star, double lambda_short )
    {
      if (0 <= z && z <= z_k_star )
      {
        return (1.0 / (1.0 - exp(-lambda_short*z_k_star))) * lambda_short * exp(- lambda_short * z);
      }
      return 0.0;
    }
    
    inline double pRand( double z, double z_max )
    {
      if (0 <= z && z <= z_max)
      {
        return 1.0 / z_max;
      }
      return 0.0;
    }
    
    std::vector<std::pair<double, double>> z_m_e_;
  };
  
};
#endif //PROJECT_SENSOR_MODEL_PARAMETER_ESTIMATOR_EM_H

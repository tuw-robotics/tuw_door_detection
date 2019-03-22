//
// Created by felix on 22.03.19.
//

#ifndef TUW_SENSOR_MODEL_PARAMETER_ESTIMATOR_EM_H
#define TUW_SENSOR_MODEL_PARAMETER_ESTIMATOR_EM_H

#include <map>
#include <vector>

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
    };
    
    void add(double measured, double expected);
  
    ParametersEstimated &compute();
  
  private:
    ParametersEstimated params_;
    
    double pHit( double z, double z_k_star, double sigma_hit );
    
    double pZmax( double z, double z_max );
    
    double pShort( double z, double z_k_star, double lambda_short );
    
    double pRand( double z, double z_max );
    
    std::vector<std::pair<double,double>> z_m_e_;
  };
  
};
#endif //PROJECT_SENSOR_MODEL_PARAMETER_ESTIMATOR_EM_H

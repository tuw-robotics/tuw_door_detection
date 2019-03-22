//
// Created by felix on 22.03.19.
//

#include <sensor_model_parameter_estimator_em.h>

using namespace tuw;

void SensorModelParameterEstimatorEM::add( double measured, double expected )
{
  z_m_e_.emplace_back( std::make_pair( measured, expected ));
}

SensorModelParameterEstimatorEM::ParametersEstimated &SensorModelParameterEstimatorEM::compute()
{
  bool do_estimate = true;
  
  do
  {
    std::vector<double> e_hit;
    std::vector<double> e_short;
    std::vector<double> e_max;
    std::vector<double> e_rand;
    
    for ( auto p_zme : z_m_e_ )
    {
      double z_kt = p_zme.first;
      double z_kt_star = p_zme.second;
      
      double z_hit = pHit(z_kt, z_kt_star, params_.sigma_hit);
      double z_short = pShort(z_kt, z_kt_star, params_.lambda_short);
      double z_max = pZmax(z_kt, params_.z_max);
      double z_rand = pRand(z_kt, params_.z_max);
      
      double nu = 1.0 / (z_hit + z_short + z_max + z_rand);
      
      e_hit.push_back(nu * z_hit);
      e_short.push_back(nu * z_short);
      e_max.push_back(nu * z_max);
      e_rand.push_back(nu * z_rand);
    }
    
    //TODO: make
    
  } while (do_estimate);
  
  
  return params_;
}

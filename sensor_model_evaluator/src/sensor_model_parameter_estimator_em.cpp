//
// Created by felix on 22.03.19.
//

#include <sensor_model_parameter_estimator_em.h>

using namespace tuw;

SensorModelParameterEstimatorEM::SensorModelParameterEstimatorEM() : params_(nullptr)
{
}

void SensorModelParameterEstimatorEM::clear()
{
  z_m_e_.clear();
}

void SensorModelParameterEstimatorEM::setParams(ParametersEstimated &params)
{
  params_ = std::make_shared<ParametersEstimated>(params);
}

void SensorModelParameterEstimatorEM::add( double measured, double expected )
{
  z_m_e_.push_back( std::make_pair( measured, expected ));
}

std::shared_ptr<SensorModelParameterEstimatorEM::ParametersEstimated> SensorModelParameterEstimatorEM::compute()
{
  if (!params_)
  {
    return nullptr;
  }
  
  printf("EM on %ld measurements\n", z_m_e_.size());
  
  //dry run
  for (auto p_zme : z_m_e_)
  {
    double z_kt = p_zme.first;
    double z_kt_star = p_zme.second;
    
    //printf("(%lf, %lf)\n", z_kt, z_kt_star);
  }
  
  bool do_estimate = true;
  do
  {
    ParametersEstimated cached_ = ParametersEstimated(*params_);
    std::vector<double> e_hit;
    std::vector<double> e_short;
    std::vector<double> e_max;
    std::vector<double> e_rand;
    double hit_normalized_total = 0;
    double short_normalized_total = 0;
    
    for ( auto p_zme : z_m_e_ )
    {
      double z_kt = p_zme.first;
      double z_kt_star = p_zme.second;
      
      double z_hit = pHit( z_kt, z_kt_star, params_->sigma_hit );
      double z_short = pShort( z_kt, z_kt_star, params_->lambda_short );
      double z_max = pZmax( z_kt, params_->z_max );
      double z_rand = pRand( z_kt, params_->z_max );
      
      printf("(%lf, %lf, %lf, %lf)\n", z_hit, z_short, z_max, z_rand);
      
      double nu = 1.0 / (z_hit + z_short + z_max + z_rand);
      
      e_hit.push_back( nu * z_hit );
      e_short.push_back( nu * z_short );
      e_max.push_back( nu * z_max );
      e_rand.push_back( nu * z_rand );
      
      hit_normalized_total += (nu * z_hit * pow(z_kt - z_kt_star, 2));
      short_normalized_total += (nu * z_short * z_kt);
    }
  
    //TODO: make
    double hit_total = 0;
    double short_total = 0;
    double max_total = 0;
    double rand_total = 0;
    for ( int k = 0; k < e_hit.size(); ++k )
    {
      hit_total += e_hit[k];
      short_total += e_short[k];
      max_total += e_max[k];
      rand_total += e_rand[k];
    }
    
    params_->z_hit = hit_total / e_hit.size();
    params_->z_short = short_total / e_short.size();
    params_->z_max = max_total / e_max.size();
    params_->z_rand = rand_total / e_rand.size();
    
    params_->sigma_hit = std::sqrt(1.0/hit_total) * hit_normalized_total;
    params_->lambda_short = short_total / short_normalized_total;
    
    double energy = cached_.dotMinus(*params_);
    printf("Energy: %lf\n", energy);
    printf("zhit: %lf\n", params_->z_hit);
    printf("zmax: %lf\n", params_->z_max);
    printf("zshort: %lf\n", params_->z_short);
    printf("zrand: %lf\n", params_->z_rand);
  
    do_estimate = energy > 0.2;
    
  } while ( do_estimate );
  
  
  return params_;
}

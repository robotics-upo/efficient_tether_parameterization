#ifndef CERES_CONSTRAINS_EQUIDISTANCE__UGV_HPP_
#define CERES_CONSTRAINS_EQUIDISTANCE__UGV_HPP_

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

#include <iostream>
#include <fstream>
#include <string>

class EquiDistanceFunctorUGV 
{
  public:
    EquiDistanceFunctorUGV(double weight_factor, double initial_distance_ugv, bool write_data, std::string user_name)
    : wf_(weight_factor), i_d(initial_distance_ugv), w_d_(write_data), user_(user_name) 
    {}

    template <typename T>
    bool operator()(const T* const statePos1, const T* const statePos2, T* residual) const 
    {  
      T arg_d_pos, d_pos, dr_;
      double f_; 
      
      //Get distance between two consecutive poses
      arg_d_pos = pow(statePos1[1]-statePos2[1],2) + pow(statePos1[2]-statePos2[2],2) + pow(statePos1[3]-statePos2[3],2);
      
      if(arg_d_pos < 0.0001 && arg_d_pos > -0.0001)
        d_pos = T{0.0};
      else
        d_pos = sqrt(arg_d_pos);

      T max_value_residual = T{10.0};
      T min_value_residual = T{0.0};
      T m;
      T init_d = T{i_d};
      
      m = (max_value_residual- min_value_residual)/( 1.5*init_d - init_d);
      
      if(d_pos > init_d)
        f_ = 1.0;
      else
        f_ = 0.0;

      residual[0] = wf_ *( m *(d_pos - init_d) + min_value_residual)*f_;  

      // std::cout<< "EquiDistance UGV: residual[0]= "<<residual[0]<<" , i_d= "<< i_d << " , d_pos= "<< d_pos <<" , init_d= ["<<init_d1<<"/"<<init_d2<<"]"<<std::endl;
    
      if(w_d_){
        std::ofstream ofs;
        std::string name_output_file = "/home/"+user_+"/residuals_optimization_data/equidistance_ugv.txt";
        ofs.open(name_output_file.c_str(), std::ofstream::app);
        if (ofs.is_open()) 
          ofs << residual[0] << "/" <<std::endl;
        ofs.close();
      }

      return true;

    }

    bool w_d_;
    double wf_, i_d;
    std::string user_;

  private:

};

#endif
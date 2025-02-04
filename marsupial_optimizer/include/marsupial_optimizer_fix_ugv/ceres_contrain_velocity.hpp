#ifndef CERES_CONTRAIN_VELOCITY_FIX_UGV_HPP
#define CERES_CONTRAIN_VELOCITY_FIX_UGV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class VelocityFunctor {

public:
  VelocityFunctor(double weight_factor, double init_vel): wf_(weight_factor), iv_(init_vel) {}

  template <typename T>
  bool operator()(const T* const state1, const T* const state2, const T* const state3, T* residual) const {

	T d_ = sqrt(pow(state2[1]-state1[1],2) + pow(state2[2]-state1[2],2) + pow(state2[3]-state1[3],2)) ;
  T dt_ = state3[1];
	T v_ = d_ / dt_;

	residual[0] =  wf_ * (v_ - iv_);

  return true;
  }

 double wf_, iv_;

 private:
};


#endif
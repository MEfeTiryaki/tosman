

#include <kulman_state_estimator/StateEstimatorNode.hpp>
#include <tosman_state_estimator/TosmanEKF.hpp>
#include <tosman_model/TosmanModel.hpp>


#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv)
{
  estimator::StateEstimatorNode<kuco::TosmanModel,estimator::TosmanEKF> estimator_ = estimator::StateEstimatorNode<kuco::TosmanModel,estimator::TosmanEKF>();
  estimator_.create();
  estimator_.initilize(argc,argv);
  estimator_.execute();
  return 0;
}

#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse = VectorXd::Zero(4);


  if(estimations.size() != ground_truth.size()
      || estimations.size() == 0 ){
    std::cout << "Estimation and Ground truth size is not the same."<< std::endl;
    return rmse;
  }

  for(int i=0;i<estimations.size();++i){
    VectorXd error = estimations[i] - ground_truth[i];
    error =  error.array() * error.array() ;
    rmse += error;
  }
  //calculate the mean
  rmse = rmse/estimations.size();

  // calculate the sqrt
  rmse = rmse.array().sqrt();

  return rmse;
}

#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

double Tools::Truncate2Pi(double v){
      size_t twoPICount = size_t(v/(2.*M_PI));
      v-=2.*M_PI * twoPICount;
      return v;
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check inputs validity:
    //  1. the estimation vector size should not be zero
    //  2. the estimation vector size should equal ground truth vector size
    if(estimations.size() == 0 || ground_truth.size() == 0 ||
            estimations.size() != ground_truth.size())
    {
        cout<<"Tools::CalculateRMSE Error: vector size mismatch"<<endl;
        return VectorXd();
    }

    //accumulate squared residuals
    for(size_t i = 0; i < estimations.size(); ++i){

        //check input vectors contain 4 values (px, py, vx, vy)
        if(estimations[i].size() != ground_truth[i].size() ||
                    rmse.size() != estimations[i].size())
        {
            cout<<"Tools::CalculateRMSE Error: vector components size mismatch"<<endl;
            return VectorXd();
        }
        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;

}

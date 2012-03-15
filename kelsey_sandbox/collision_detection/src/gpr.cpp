/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
  Author: Daniel Hennes
 */

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include <iostream>
#include <fstream>
#include <vector>

#include "inverse_dynamics/utils.h"

#include <optimization.h>

#define _USE_MATH_DEFINES
#include <cmath>
 
using namespace Eigen;

class GPRModel {
 
private:
  MatrixXd X; // support set
  MatrixXd Y;

  VectorXd alpha; // prediction vector

  std::vector<double> hyp; // hyperparameters

  double nlZ; // negative log-likelihood
  std::vector<double> dnlZ; // derivative of neg. log-likelihood

  MatrixXd k(MatrixXd A, MatrixXd B); // kernel function

public:
  GPRModel();
  void setParameters();
  void inference(MatrixXd X, VectorXd Y);
  double predict(VectorXd x);
};

GPRModel::GPRModel() {
  hyp.resize(3);
  hyp[0] = 0.0; // log(ell), lengthscale
  hyp[1] = 0.0; // log(sigma_f^2)
  hyp[2] = log(0.1); // log(sigma_n^2)
  nlZ = 0.0;
  dnlZ.resize(3);
}

void GPRModel::setParameters(std::vector<double> params) {
  hyp = params;
}

std::vector<double> GPRModel::getHyp() {
  return hyp;
}

MatrixXd GPRModel::k(MatrixXd A, MatrixXd B) {
  /* 
     implements the following kernel function
     K = k(X,X)
     k = sf^2 * exp(-(x^p - x^q)'*inv(P)*(x^p - x^q)/2) 
     P = eye * el1^2
  */

  // lengthscale

  double ell = exp(hyp[0]);
  double sf2 = exp(hyp[1] * 2.0);

  A /= ell;
  B /= ell;

  MatrixXd K = MatrixXd::Zero(A.rows(), B.rows());

  // compute squared distance: (a-b)^2
  // a^2
  MatrixXd x = A.array().pow(2).rowwise().sum();
  for (int i = 0; i < K.cols(); i++) {
    K.col(i) += x;
  }
  // b^2
  x = B.array().pow(2).rowwise().sum().transpose();
  for (int i = 0; i < K.rows(); i++) {
    K.row(i) += x;
  }
  // -2 * a * b
  K -= 2.0 * A * B.transpose();

  // eliminate numerical erros (negative entries in matrix)
  K = K.array().abs();

  K /= -2.0;
  K = K.array().exp() * sf2;

  return K;
}

MatrixXd sq_dist(MatrixXd A, MatrixXd B) {
  MatrixXd K = MatrixXd::Zero(A.rows(), B.rows());

  // compute squared distance: (a-b)^2
  // a^2
  MatrixXd x = A.array().pow(2).rowwise().sum();
  for (int i = 0; i < K.cols(); i++) {
    K.col(i) += x;
  }
  // b^2
  x = B.array().pow(2).rowwise().sum().transpose();
  for (int i = 0; i < K.rows(); i++) {
    K.row(i) += x;
  }
  // -2 * a * b
  K -= 2.0 * A * B.transpose();

  // eliminate numerical erros (negative entries in matrix)
  K = K.array().abs();

  return K;
}

VectorXd GPRModel::inference(MatrixXd X, VectorXd Y) {
  double ell = exp(hyp[0]);
  double sf2 = exp(hyp[1] * 2.0); 
  double sn2 = exp(hyp[3] * 2.0);

  // printf("[infExact] called with: %5.3f, %5.3f, %5.3f\n", ell, sf2, sn2);

  int n = X.rows();
  int D = X.cols();

  VectorXd alpha;
 
  // evaluate covariance matrix
  MatrixXd K;
  K = k(X);
  
  // evaluate mean vector
  VectorXd m = VectorXd::Zero(n);

  // Cholesky factor of covariance with noise
  Eigen::LLT<MatrixXd> LLT;
  LLT = (K / sn2 + MatrixXd::Identity(n, n)).llt();
  alpha = LLT.solve(Y - m);
  alpha /= sn2;

  // negative marginal log-likelihood
  MatrixXd L = LLT.matrixL();

  hyp.nlZ = (((Y - m).transpose() * alpha / 2.0).array() 
             + L.diagonal().array().log().sum())[0] + n * log(2 * M_PI * sn2) / 2.0;
  // printf("negative log-likelihood: %6.4f\n", nlZ);
  printf("[infExact] nlZ: %.5f\n", hyp.nlZ);

  // squared distance
  // copy from function k
  K = sq_dist(X / ell, X / ell);

  // derivatives
  MatrixXd Q = LLT.solve(MatrixXd::Identity(n, n)) / sn2 - alpha * alpha.transpose();

  // hyp.ell
  MatrixXd K1 = sf2 * (K / -2.0).array().exp() * K.array();
  hyp.dnlZ[0] = (Q.array() * K1.array()).sum() / 2.0;
  // printf("dnlZ(ell): %6.4f\n", hyp.dnlZ[0]);

  // hyp.sf2
  MatrixXd K2 = 2.0 * sf2 * (K / -2.0).array().exp();
  hyp.dnlZ[1] = (Q.array() * K2.array()).sum() / 2.0;
  // printf("dnlZ(sf2): %6.4f\n", hyp.dnlZ[1]);

  // hyp.sn2
  hyp.dnlZ[2] = sn2 * Q.diagonal().sum();
  // printf("dnlZ(sn2): %6.4f\n", hyp.dnlZ[2]);
  
  return alpha;
}

void cb_minimize(const alglib::real_1d_array &x, double &func, alglib::real_1d_array &grad, void *ptr) {
  hyp.cov[0] = x[0];
  hyp.cov[1] = x[1];
  hyp.lik = x[2];
  VectorXd alpha = infExact(myX, myY);
  func = hyp.nlZ;
  grad[0] = hyp.dnlZ[0];
  grad[1] = hyp.dnlZ[1];
  grad[2] = hyp.dnlZ[2];
}


double GPRModel::predict(VectorXd x) {
  MatrixXd ks;
  MatrixXd xs;
  xs = x.transpose();
  ks = k(X, xs);
  VectorXd v = (ks.transpose() * alpha);
  return v(0);
}

int main(int, char *[])
{
  MatrixXd data;
  utils::readMatrix("../data/test.txt", &data);

  // data.col(0);
  // data.col(1);

  alglib::real_1d_array x="[0,0,0]";
  x[0] = hyp.cov[0];
  x[1] = hyp.cov[1];
  x[2] = hyp.lik;
  double epsg = 0.00001;
  double epsf = 0;
  double epsx = 0;
  alglib::ae_int_t maxits = 100;
  alglib::minlbfgsstate state;
  alglib::minlbfgsreport rep;

  alglib::minlbfgscreate(1, x, state);
  alglib::minlbfgssetcond(state, epsg, epsf, epsx, maxits);
  alglib::minlbfgsoptimize(state, cb_minimize);
  alglib::minlbfgsresults(state, x, rep);

  int const tt = int(rep.terminationtype);
  switch (tt) {
  case 4:
    printf("[minlbfgs] gradient norm <= %.5f.\n", epsg);
    break;
  case 5:
    printf("[minlbfgs] max # iterations (%d) reached.\n", int(maxits));
    break;
  }
  printf("\nhyperparams: %.5f, %.5f, %.5f\n", hyp.cov[0], hyp.cov[1], hyp.lik);

}

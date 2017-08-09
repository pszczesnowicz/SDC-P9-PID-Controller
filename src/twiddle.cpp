#include <iostream>
#include <vector>
#include <numeric>
#include <math.h>

#include "twiddle.h"

using namespace std;

Twiddle::Twiddle() {
  
  i = 0;
  
  order = 1;
  
  iterations = 0;
  
  best_iterations = 0;
  
  sum_squared_error = 0.0;
  
  best_error = 100.0;
  
}


Twiddle::~Twiddle() {}


void Twiddle::Init(double Kp_inc, double Ki_inc, double Kd_inc, double tol) {
  
  gain_increments.resize(3);
  
  gain_increments[0] = Kp_inc;
  gain_increments[1] = Ki_inc;
  gain_increments[2] = Kd_inc;
  
  tolerance = tol;
  
}


// Calculate the sum of the twiddle parameter tuning vector
double Twiddle::CalculateSum() {
  
  return accumulate(gain_increments.begin(), gain_increments.end(), 0.0);
  
}

// Calculate the root mean squared error
double Twiddle::CalculateError() {
  
  return sqrt(sum_squared_error / iterations);
  
}

void Twiddle::ResetError() {
  
  iterations = 0;
  sum_squared_error = 0.0;
  
}

#include <iostream>
#include <vector>
#include <math.h>

#include "PID.h"

using namespace std;

// Constructor
PID::PID() {
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  d_error_past = 0.0;

}

// Destructor
PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  gain.resize(3);
  
  gain[0] = Kp;
  gain[1] = Ki;
  gain[2] = Kd;
  
}

void PID::UpdateError(double cte) {
  
  // Proportional
  p_error = cte;
  
  // Integral
  if (cte < 0.001) {
    i_error = 0;
  }
  else {
    i_error += cte;
  }
  
  // Derivative
  d_error = cte - d_error_past;
  d_error_past = d_error;
  
}

void PID::ResetError() {
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  d_error_past = 0.0;
  
}

double PID::TotalError() {
  
  double total_error;
  
  total_error = -gain[0] * p_error - gain[1] * i_error - gain[2] * d_error;
  
  if (total_error > 1.0) {
    total_error = 1.0;
  }
  else if (total_error < -1.0) {
    total_error = -1.0;
  }
  
  return total_error;
  
}


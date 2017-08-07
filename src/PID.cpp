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
  
  p_error = cte;
  i_error += cte;
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
  
  return -gain[0] * p_error - gain[1] * i_error - gain[2] * d_error;
  
}


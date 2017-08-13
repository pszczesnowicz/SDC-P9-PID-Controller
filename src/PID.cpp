#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>

#include "PID.h"

using namespace std;


// Constructor
PID::PID() {}


// Destructor
PID::~PID() {}


// Initializes the PID controller
void PID::Init(double Kp, double Ki, double Kd,
               double Kp_inc, double Ki_inc, double Kd_inc) {
  
  // PID gains
  gains.resize(3);
  gains[0] = Kp;
  gains[1] = Ki;
  gains[2] = Kd;
  
  // PID errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  cte_past = 0.0;
  
  // PID gain increments used for tuning
  gain_increments.resize(3);
  gain_increments[0] = Kp_inc;
  gain_increments[1] = Ki_inc;
  gain_increments[2] = Kd_inc;
  
  // Index of the gain being tuned
  i = 0;
  
  // Tuning order
  order = 1;
  
  // Accumulated mean squared error variables
  iterations = 0;
  sum_squared_error = 0.0;
  best_error = 1.0;
  
}


// Updates the PID errors given cross track error
void PID::UpdateError(double error) {
  
  // Proportional error
  p_error = error;
  
  // Integral error
  i_error += error;

  // Derivative error
  d_error = error - cte_past;
  cte_past = error;
  
  // Accumulated mean squared error
  iterations += 1;
  sum_squared_error += pow(error, 2.0);
  
}


// Calculates the total PID error
double PID::TotalError() {
  
  return gains[0] * p_error + gains[1] * i_error + gains[2] * d_error;
  
}


// Calculates the sum of the PID gain increments
double PID::CalculateSum() {
  
  return accumulate(gain_increments.begin(), gain_increments.end(), 0.0);
  
}


// Calculates the accumulated mean squared error
double PID::CalculateError() {
  
  if (iterations == 0) {
    return 0;
  }
  
  else {
    return sum_squared_error / iterations;
  }
  
  
}


// Resets the PID and accumulated mean squared errors
void PID::ResetError() {
  
  // PID errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  cte_past = 0.0;
  
  // Accumulated mean squared error
  iterations = 0;
  sum_squared_error = 0.0;
  
}


// Increments the index of the PID gain being tuned
// Skips the integral gain
// Resets the tuning order
void PID::IncrementIndex() {
  
  // Moving to the next PID gain
  i = (i + 1) % 3;
  
  // Skipping the integral gain
  // The simulator does not have a steering bias
  if (i == 1) {
    i += 1;
  }
  
  // Resetting the order for the next PID gain
  order = 1;
  
}


// Tunes the PID gains
void PID::Twiddle() {
  
  switch (order) {
      
    case 1: {
      
      // Checking if the the previous twiddle improved the error and distance traveled
      if (CalculateError() < best_error) {
        
        // Setting new improvement requirement
        best_error = CalculateError();
        
        // Increasing the PID gain incrementing value
        gain_increments[i] *= 1.1;
        
        // Moving to the next PID gain and resetting the tuning order
        IncrementIndex();
        
        // Incrementing the PID gain
        gains[i] += gain_increments[i];
        
        break;
        
      }
      
      else {
        
        // Twiddling in the other direction if the previous twiddle did not improve performance
        gains[i] -= 2 * gain_increments[i];
        
        // Ensuring that the PID gain stays positive
        if (gains[i] < 0) {
          gains[i] = 0.0;
        }
        
        // On the next twiddle check the effect of this twiddle
        order = 2;
        
        break;
        
      }
      
    } // End case 1
      
    case 2: {
      
      // Checking if the the previous twiddle improved the error and distance traveled
      if (CalculateError() < best_error) {
        
        // Setting new improvement requirements
        best_error = CalculateError();
        
        // Increasing the PID gain incrementing value
        gain_increments[i] *= 1.1;
        
        // Moving to the next PID gain and resetting the tuning order
        IncrementIndex();
        
        // Incrementing the PID gain
        gains[i] += gain_increments[i];
        
        break;
        
      }
      
      else {
        
        // Resetting the PID gain to its value before the first twiddle
        gains[i] += gain_increments[i];
        
        // Decreasing the PID gain incrementing value
        gain_increments[i] *= 0.9;
        
        // Moving to the next PID gain and resetting the tuning order
        IncrementIndex();
        
        // Incrementing the PID gain
        gains[i] += gain_increments[i];

        break;
        
      }
      
    } // End case 2
      
  } // End switch
  
  // Resetting the accumulated mean squared error
  iterations = 0;
  sum_squared_error = 0.0;
  
}

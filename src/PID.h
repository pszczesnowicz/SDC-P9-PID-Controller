#ifndef PID_H
#define PID_H

class PID {

private:
  
  // PID errors
  double p_error;
  double i_error;
  double d_error;
  double cte_past;
  
  // Increments the index of the PID gain being tuned
  // Skips the integral gain
  // Resets the tuning order
  void IncrementIndex();
  
public:
  
  // PID gains
  std::vector<double> gains;
  
  // PID gain increments used for tuning
  std::vector<double> gain_increments;
  
  // Index of the PID gain being tuned
  int i;
  
  // Tuning order
  int order;
  
  // Accumulated mean squared error variables
  int iterations;
  double sum_squared_error;
  double best_error;
  
  // Constructor
  PID();

  // Destructor
  virtual ~PID();

  // Initializes the PID controller
  void Init(double Kp, double Ki, double Kd,
            double Kp_inc, double Ki_inc, double Kd_inc);

  // Updates the PID errors given cross track error
  void UpdateError(double error);
  
  // Calculates the total PID error
  double TotalError();
  
  // Calculates the sum of the PID gain increments
  double CalculateSum();
  
  // Calcuates the accumulated mean squared error
  double CalculateError();
  
  // Resets the PID and accumulated mean squared errors
  void ResetError();
  
  // Tunes the PID gains
  void Twiddle();
  
};

#endif // PID_H

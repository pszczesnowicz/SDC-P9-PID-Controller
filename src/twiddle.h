#ifndef twiddle_H
#define twiddle_H

class Twiddle {
public:
  
  // Parameter tuning vector
  std::vector<double> gain_increments;
  
  // Parameter tuning vector tolerance
  double tolerance;
  
  // Error calculating variables
  int i;
  int order;
  int iterations;
  int best_iterations;
  double sum_squared_error;
  double best_error;

  // Constructor
  Twiddle();
  
  // Destructor
  virtual ~Twiddle();
  
  // Initialize Twiddle
  void Init(double Kp_inc, double Ki_inc, double Kd_inc, double tol);
  
  
  /**
   Calculates the sum of the gain tuning vector

   @return sum of gain tuning vector
   */
  double CalculateSum();
  
  
  /**
   Calculates the mean squared error

   @return root mean squared error
   */
  double CalculateError();
  
  
  /**
   Resets the variables used to calculate error
   */
  void ResetError();
  
};

#endif // twiddle_H

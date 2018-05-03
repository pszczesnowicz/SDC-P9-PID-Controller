This is my submission for the Udacity Self-Driving Car Nanodegree PID Controller Project. You can find my C++ source code [here](https://github.com/pszczesnowicz/SDCND-P9-PIDController/tree/master/src). The goal of this project was to utilize PID controllers to drive a car around a simulated track. The same task was accomplished using Deep Learning by my [Behavioral Cloning Project](https://pszczesnowicz.github.io/SDC-P3-Behavioral-Cloning/).

# Summary

I used one PID controller for steering and another for throttle. The PID gains were tuned automatically while the car drove around the track using the twiddle method.

# PID Gains

PID stands for proportional, integral, and derivative:

* Proportional - As the name implies, this controller's response is proportional to the current error.

* Integral - This controller counters continuous error, such as steering bias, by accumulating it over time.

* Derivative - This controller tries to predict and counter future error based on its current rate of change, therefore it will dampen the response of the proportional controller.

The error is the difference between a target and measured value. The error for the steering controller is the difference between the target trajectory and car's center, and for the throttle controller it is the difference between the target and car's speed. Each controller is multiplied by an appropriate gain that governs the magnitude of the response.

Because the car in the simulator does not have any steering bias, I set the steering controller's integral gain to 0. PID gain optimization yielded 0.0933784 and 1.64645 for the steering controller's proportional and derivative gains, respectively. I manually set the throttle controller's proportional, integral, and derivative gains to 0.2, 0, and 3.0, respectively. During testing, these gains kept the car's speed fairly constant.

# PID Gain Optimization

To improve the performance of the steering controller I implemented the twiddle method to tune the gains. The logic behind the twiddle method is as follows:

1. Increment the gain (first twiddle).

2. After a number of iterations check whether the first twiddle improved performance.

3. If the performance improved, then increase the increment amount and move onto the next gain (go to 1).

4. If the performance did not improve, then decrement the gain twice (second twiddle).

5. After a number of iterations check whether the second twiddle improved performance.

6. If the performance improved, then increase the increment amount and move onto the next gain (go to 1).

7. If the performance did not improve, then increment the gain once to return it to its original value, decrease the increment amount, and move onto the next gain (go to 1).

I measured the performance of the PID gains using an accumulated mean squared error and saved the best error each time it decreased.

While optimizing the steering gains, the target speed was set to 40 MPH and was held fairly constant by the throttle controller. Once the steering gains were optimized, the target speed was increased to 60 MPH. To help keep the car steady, the throttle controller's output was multiplied by the inverse of the current steering angle: with an increase in steering angle, there is a decrease in throttle proportional to the turn sharpness.

# Results

Because of the inverse relationship of the throttle and steering, the car never reached 60 MPH, but managed a top speed of 55 MPH while staying in control.

[<img src="https://raw.githubusercontent.com/pszczesnowicz/SDCND-P9-PIDController/master/readme_images/pid_controller.jpg" width="800">](https://youtu.be/EKCI0xIOJRA")

# Conclusion

The PID controller is limited in this application because it does not have information about the planned trajectory and can only try to anticipate changes to the car's path with the derivative controller.

This project is succeeded by my [Model Predictive Control Project](https://pszczesnowicz.github.io/SDC-P10-Model-Predictive-Control/) which is able to keep the car in control at higher speeds with smoother steering.

# References

[Udacity Self-Driving Car ND](http://www.udacity.com/drive)

[Udacity Self-Driving Car ND - PID Controller Project](https://github.com/udacity/CarND-PID-Control-Project)

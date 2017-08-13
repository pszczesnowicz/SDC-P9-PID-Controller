#include <iostream>
#include <vector>
#include <math.h>
#include <uWS/uWS.h>

#include "json.hpp"
#include "PID.h"

using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  
  if (found_null != std::string::npos) {
    return "";
  }
  
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  
  return "";
}

void ResetSimulator(uWS::WebSocket<uWS::SERVER> ws) {
  
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  
}

int main()
{
  uWS::Hub h;
  
  // Creating PID controllers
  PID pid_steering, pid_throttle;
  
  // Initializing the PID controllers
  // Kp, Ki, Kd, Kp_inc, Ki_inc, Kd_inc
  pid_steering.Init(0.05, 0.0, 0.0, 0.05, 0.0, 0.5);
  pid_throttle.Init(0.2, 0.0, 3.0, 0.0, 0.0, 0.0);
  
  int total_iterations;
  total_iterations = 0;
  
  h.onMessage([&pid_steering, &pid_throttle, &total_iterations]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
                
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      
      auto s = hasData(std::string(data).substr(0, length));
      
      // Start autonomous mode
      if (s != "") {
        
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          
          double steer_value, throttle_value, target_speed, speed_error;
          
          pid_steering.UpdateError(cte);
          
          // Normalizing the steering value
          steer_value = pid_steering.TotalError() / -deg2rad(25.0);
          
          total_iterations += 1;
          
          // Starting optimizing mode if the sum of the gain increments is greater than the set threshold
          if (pid_steering.CalculateSum() > 0.1) {
            
            cout << "Optimizing" << endl;
            
            // Optimizing the PID gains while trying to maintain a constant speed
            target_speed = 40;
            speed_error = target_speed - speed;
            pid_throttle.UpdateError(speed_error);
            throttle_value = pid_throttle.TotalError();
            
            // Resetting the simulator if the car drives off the track or gets stuck
            if ((fabs(cte) > 4.5 && total_iterations > 100) || (speed < 5.0 && total_iterations > 100)) {
              
              cout << "Resetting" << endl;
              
              pid_steering.Twiddle();
              ResetSimulator(ws);
              pid_steering.ResetError();
              total_iterations = 0;
              
              // Skipping output to simulator
              goto end_cycle;
              
            }
            
            else if (total_iterations > 400) {
              
              cout << "Twiddling" << endl;
              
              pid_steering.Twiddle();
              total_iterations = 0;
              
            }
            
            cout << "Best Error: " << pid_steering.best_error << " Current Error: " << pid_steering.CalculateError() << endl;
            
            switch (pid_steering.i) {
                
              case 0: {
                cout << "Tuning Proportional Gain - Twiddle Order: " << pid_steering.order << endl;
                break;
              }
                
              case 1: {
                cout << "Tuning Integral Gain - Twiddle Order: " << pid_steering.order << endl;
                break;
              }
                
              case 2: {
                cout << "Tuning Derivative Gain - Twiddle Order: " << pid_steering.order << endl;
                break;
              }
                
            } // End switch
            
            cout << "P inc: " << pid_steering.gain_increments[0] << " I inc: " << pid_steering.gain_increments[1] << " D inc: " << pid_steering.gain_increments[2] << endl;
            
          } // End optimizing mode
          
          else {
            
            cout << "Optimized" << endl;
            
            target_speed = 60;
            speed_error = target_speed - speed;
            pid_throttle.UpdateError(speed_error);
            throttle_value = pid_throttle.TotalError() * (1.0 / (1.0 + fabs(angle)));
            
          }
          
          cout << "P: " << pid_steering.gains[0] << " I: " << pid_steering.gains[1] << " D: " << pid_steering.gains[2] << endl;
          cout << "CTE: " << cte << " Steering Value: " << rad2deg(steer_value) << " degrees" << endl;
          cout << "Throttle Value: " << throttle_value << endl;
          cout << endl;
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
        
      } // End autonomous mode
      
      // Start manual mode
      else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        
      } // End manual mode
      
    }
    
  end_cycle:
    ;
                
  });

  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    }
    else {
      res->end(nullptr, 0);
    }
    
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}

#include <iostream>
#include <vector>
#include <math.h>
#include <uWS/uWS.h>

#include "json.hpp"
#include "PID.h"
#include "twiddle.h"

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
  
  // Initializing the PID variable
  PID pid;
  
  pid.Init(0.05, 0.0001, 0.5);
  
  // Initializing the twiddle variable
  Twiddle twiddle;
  
  twiddle.Init(0.01, 0.0, 0.01, 1);
  
  h.onMessage([&pid, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value = 0;
          
          // Start optimizing mode
          if (twiddle.CalculateSum() > twiddle.tolerance) {
            
            cout << "Optimizing" << endl;
            
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            
            twiddle.iterations += 1;
            twiddle.sum_squared_error += pow(cte, 2.0);
            
            // Resetting the simulator if the car drives off the track or gets stuck
            // Start twiddle
            if ((fabs(cte) > 4.5 && twiddle.iterations > 200)|| (speed < 5 && twiddle.iterations > 200)) {
              
              switch (twiddle.order) {
                
                // First twiddle - Incrementing the PID gain
                case 1: {
                  cout << "first twiddle **************************************************************" << endl;
                  
                  pid.gain[twiddle.i] += twiddle.gain_increments[twiddle.i];
                  
                  twiddle.order = 2;
                  break;
                  
                } // End case 1
                
                // Second twiddle - Checking the effect of the first twiddle
                case 2: {
                  cout << "second twiddle **************************************************************" << endl;
                  
                  // Checking if the the first twiddle improved the error and distance traveled
                  if (twiddle.CalculateError() < twiddle.best_error && twiddle.iterations > twiddle.best_iterations) {
                    
                    // Setting new improvement requirements
                    twiddle.best_error = twiddle.CalculateError();
                    twiddle.best_iterations = twiddle.iterations;
                    
                    // Increasing the PID gain incrementing value
                    twiddle.gain_increments[twiddle.i] *= 1.1;
                    
                    // Resetting the order for the next PID gain
                    twiddle.order = 1;
                    
                    // Moving to the next PID gain
                    twiddle.i = (twiddle.i + 1) % 3;
                    
                    // Skipping the integral gain - Best performance with integral term = 0 so far
                    if (twiddle.i == 1) {
                      twiddle.i += 1;
                    }
                    
                    break;
                    
                  }
                  
                  else {
                    
                    // If the first twiddle did not improve performance then twiddle in the other direction
                    pid.gain[twiddle.i] -= 2 * twiddle.gain_increments[twiddle.i];
                    
                    // Check to ensure that the PID gains stay positive
                    if (pid.gain[twiddle.i] < 0) {
                      pid.gain[twiddle.i] = 0.0;
                    }
                    
                    // On next simulator crash check effect of second twiddle
                    twiddle.order = 3;
                    break;
                    
                  }
                  
                } // End case 2
                
                // Third twiddle - Checking the effect of the second twiddle
                case 3: {
                  cout << "third twiddle **************************************************************" << endl;
                  
                  // Checking if the the second twiddle improved the error and distance traveled
                  if (twiddle.CalculateError() < twiddle.best_error && twiddle.iterations > twiddle.best_iterations) {
                    
                    // Setting new improvement requirements
                    twiddle.best_error = twiddle.CalculateError();
                    twiddle.best_iterations = twiddle.iterations;
                    
                    // Increasing the PID gain incrementing value
                    twiddle.gain_increments[twiddle.i] *= 1.1;
                    
                    // Resetting the order for the next PID gain
                    twiddle.order = 1;
                    
                    // Moving to the next PID gain
                    twiddle.i = (twiddle.i + 1) % 3;
                    
                    // Skipping the integral gain - Best performance with integral term = 0 so far
                    if (twiddle.i == 1) {
                      twiddle.i += 1;
                    }
                    
                    break;
                    
                  }
                  
                  else {
                    
                    // Resetting the PID gain to its value before the first twiddle
                    pid.gain[twiddle.i] += twiddle.gain_increments[twiddle.i];
                    
                    // Decreasing the PID gain incrementing value
                    twiddle.gain_increments[twiddle.i] *= 0.9;
                    
                    // Resetting the order for the next PID gain
                    twiddle.order = 1;
                    
                    // Moving to the next PID gain
                    twiddle.i = (twiddle.i + 1) % 3;
                    
                    // Skipping the integral gain - Best performance with integral term = 0 so far
                    if (twiddle.i == 1) {
                      twiddle.i += 1;
                    }
                    
                    break;
                    
                  }
                  
                } // End case 3
 
              } // End switch
              
              cout << "Resetting" << endl;
              
              ResetSimulator(ws);
              pid.ResetError();
              twiddle.ResetError();
              
              // Skip sending control to simulator
              goto end_cycle;
              
            } // End twiddle
            
          } // End optimizing mode
          
          // Calculating the steering angle using the optimized PID gain values
          // Start optimized mode
          else {
            
            cout << "Optimized" << endl;
            
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            
            twiddle.iterations += 1;
            twiddle.sum_squared_error += pow(cte, 2.0);
            
          } // End optimized mode
          
//          cout << "PID gain: " << twiddle.i << endl;
//          cout << "Twiddle order: " << twiddle.order << endl;
//          cout << "P: " << pid.gain[0] << " I: " << pid.gain[1] << " D: " << pid.gain[2] << endl;
//          cout << "P inc: " << twiddle.gain_increments[0] << " I inc: " << twiddle.gain_increments[1] << " D inc: " << twiddle.gain_increments[2] << endl;
          
          cout << "Error: " << twiddle.CalculateError() << endl;
          
          cout << "CTE: " << cte << " Steering Value: " << steer_value << endl;
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //cout << msg << endl;
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
    int j = 0;
    ++j;
    
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

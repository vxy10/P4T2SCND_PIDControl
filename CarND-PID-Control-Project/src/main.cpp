#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "filter.h"
#include "deadband.h"
#include "twiddle.h"
#include <math.h>

using Eigen::VectorXd;
using namespace std;

// for convenience
using json = nlohmann::json;

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

int main()
{
  uWS::Hub h;

  PID pid;
  deadband DB_steer;
  deadband DB_cte;
  filter FIL_str;
  filter FIL_throttle;
  twiddle twiddle_tune;
  // TODO: Initialize the pid variable.

  VectorXd Params = VectorXd(5);

  double Kp = 0.1;
  double Ki = 0.0004;
  double Kd = 2;
  double collecting = 1;
  double increase  = 1;
  double set_speed =75;
  double max_speed = 0;



  Params<<Kp,Ki,Kd,.2,4;
  cout<<Params;
  twiddle_tune.Init(0.8,100,set_speed,Params);

  pid.Init(Kp, Ki, Kd); // Values taken from lecture

  DB_steer.Init(0.25,40);
  DB_cte.Init(0.25,20);
  FIL_str.Init(.99);
  FIL_throttle.Init(.99);
  twiddle_tune.changeParam(1,Params);

  h.onMessage([&pid,&DB_steer, &DB_cte, &FIL_str,&twiddle_tune,&Params,&Kp,&Kd,&Ki,
    &collecting,&increase,&set_speed,&max_speed,
    &FIL_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].

          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);

          steer_value = pid.TotalError();
          steer_value = FIL_str.smooth(steer_value);

          FIL_str.savePrevious(steer_value);

          double throtte_value = 0.3;
          throtte_value = (set_speed - speed)*Params[3] - cte*Params[4];

          throtte_value += DB_steer.deadband_control(steer_value);
          throtte_value += DB_cte.deadband_control(cte);

          //throtte_value = 0.3;

          throtte_value = FIL_throttle.smooth(throtte_value);

          if (max_speed<speed){max_speed = speed;}


          if (throtte_value>1){throtte_value = 1;}
          if (throtte_value<-1){throtte_value = -1;}
          if (speed<35){throtte_value = 0.35;}
          FIL_throttle.savePrevious(throtte_value);
          //throtte_value = 0.3;

          twiddle_tune.calcError(cte,speed);


          if (twiddle_tune.countIter()==40){

            Params = twiddle_tune.updateparameters();
            Kp = Params[0];
            Ki = Params[1];
            Kd = Params[2];
            pid.Init(Kp, Ki, Kd);
            twiddle_tune.setCount(0);
            cout<<"Max_Speed = "<<max_speed<<endl;


          }




          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throtte_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

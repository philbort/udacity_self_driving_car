#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"

using namespace std;
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos)
    return "";
  else if (b1 != string::npos && b2 != string::npos)
    return s.substr(b1, b2 - b1 + 1);
  return "";
}

static const double commanded_speed = 35;

int main()
{
  uWS::Hub h;

  PID steer_pid;
  PID throttle_pid(1.0, 0.004, 3.0);

  h.onMessage([&steer_pid, &throttle_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data));
      if (s != "")
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          const double steering_cte = stod(j[1]["cte"].get<string>());
          const double speed = stod(j[1]["speed"].get<string>());
          const double speed_err = speed - commanded_speed;
          const double angle = stod(j[1]["steering_angle"].get<string>());
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          steer_pid.UpdateError(steering_cte);
          double steer_value = steer_pid.TotalError();
          
          throttle_pid.UpdateError(speed_err);
          double throttle_value = throttle_pid.TotalError();

          // DEBUG
          cout << "Steering CTE: " << steering_cte << " Steering Value: " << steer_value << endl;
          cout << "Throttle Error: " << speed_err << " Throttle Value: " << throttle_value << endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          cout << msg << endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } 
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    const string s = "<h1>Hello world!</h1>";
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
  {
    ws.close();
    cout << "Disconnected" << endl;
  });

  const int port = 4567;

  if (h.listen(port))
    cout << "Listening to port " << port << endl;
  else
  {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}

#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double system_delay = 0.1; // seconds
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // Convert veolcity from mph to m/s
          double velocity = v*1.6*5/18;

          // get current actuator values
          double steer_value=j[1]["steering_angle"];
          // std::cout<< "steer_value: "<< steer_value << std::endl;
          // Review notes: This value is in correct range, Don't need to scale it
          // steer_value= -1*steer_value * deg2rad(25);
          steer_value= -1*steer_value;
          // std::cout<< "steer sim value: "<< steer_value << std::endl;
          double throttle_value=j[1]["throttle"];;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // double Lf = 2.67;
          // Step 1: Since there is a system delay, predict where the car will be after that delay:
          // NOTE: We are recreating the input dela, i.e the time it takes for the actuators to
          // act on the received command in real world. So the command we genrated will get applied to the
          // vehicle after this delay.
          px = px + velocity * cos(psi) * system_delay;
          py = py + velocity * sin(psi) * system_delay;
          psi = psi + (velocity/Lf) * steer_value * system_delay;
          velocity = velocity + throttle_value * system_delay;


          // Step 2:  Now that we have accounted for any potential delay,
          // Change coordinates of all incoming waypoints from world to body.
          std::vector<double> body_ptsx(ptsx.size());
          std::vector<double> body_ptsy(ptsx.size());
          for (int i=0; i<ptsx.size(); i++){
            double new_x = ptsx[i] - px;
            double new_y = ptsy[i] - py;
            double c0 = cos(psi);
            double s0 = sin(psi);
            body_ptsx[i] =  c0 * new_x + s0 * new_y;
            body_ptsy[i] = -s0 * new_x + c0 * new_y;
          }


          // Step 3: Fit a 3rd degree polynomial to get trajectory based on current body reference frame waypoints.

          // Convert the std::vector to Eigen::VectorXd,
          // since this is needed by the polyfit function
          double *ptrx = &body_ptsx[0];
          double *ptry = &body_ptsy[0];
          Eigen::Map<Eigen::VectorXd> eig_ptsx(ptrx,body_ptsx.size());
          Eigen::Map<Eigen::VectorXd> eig_ptsy(ptry,body_ptsy.size());

          auto fit_coeffs = polyfit(eig_ptsx,eig_ptsy,3);

          // Step 4: Caculate the current cross track error, heading angle error and create the current state vector.
          // NOTE: Since we have done the change of coordinates, cte wil lbe evaluated at x=0;
          // And for the heading error, the ego_psy value will be 0 degrees.
          // So the new state vector will be 0,0,0,v,cte,epsi

          // Calculate the cross track error
          double cte = polyeval(fit_coeffs,0.0);

          // Calculate the orientation error:
          // double ddx_fx = fit_coeffs[1] + 2*fit_coeffs[2]*px + 3*fit_coeffs[3]*px*px;
          // Since px=0, and psi=0
          double epsi = 0.0 - atan(fit_coeffs[1]);
          // std::cout<< "epsi: "<< epsi <<std::endl;
          // std::cout<< "cte: "<< cte <<std::endl;

          Eigen::VectorXd current_states(6);
          current_states<< 0.0, 0.0, 0.0, velocity, cte, epsi;

          // Step 5: Cal the MPC Solver to get the new input commands and calculated trajectory
          // Call the MPC solver.
          auto results = mpc.Solve(current_states,fit_coeffs);
          // results format: steering, throttle, and (alternating predicted_x,predicted_y)
          // NOTE: Remember that these values were calculated for 100 millisec in future.
          // By the time this command reaches the simulator, 100 mSec will pass, and the command will be applied at the right time.

          // Reverse sign of steer_value. simulator expects reverse.
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].

          steer_value = -1*results[0]/deg2rad(25);
          throttle_value = results[1];

          // Step 6: Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          // We have the polyfit coeffs already. So lets use that to build up a reference trajectory
          double delta_dist = 2;
          int num_points = 25; // Lets see the path for 50 meters ahead of us.
          for (int i =1;i<num_points; i++){
            double curr_x = 0.0 + i*delta_dist;
            next_x_vals.push_back(curr_x);
            next_y_vals.push_back(polyeval(fit_coeffs, curr_x));

          }

          // Step 7: Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (int i =2;i<results.size();i++){
            if (i%2==0){
              mpc_x_vals.push_back(results[i]);
            }
            if (i%2==1){
              mpc_y_vals.push_back(results[i]);
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          int sleep_millis = system_delay * 1000;
          this_thread::sleep_for(chrono::milliseconds(sleep_millis));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

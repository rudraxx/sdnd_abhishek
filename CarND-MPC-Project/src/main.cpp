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

          // std::cout << "My ptsx is: "<< ptsx[0] <<std::endl;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          std::vector<double> body_ptsx(ptsx.size());
          std::vector<double> body_ptsy(ptsx.size());
          // Change coordinates of all incoming waypoints from world to body.
          for (int i=0; i<ptsx.size(); i++){
            double new_x = ptsx[i] - px;
            double new_y = ptsy[i] - py;
            double c0 = cos(psi);
            double s0 = sin(psi);
            body_ptsx[i] =  c0 * new_x + s0 * new_y;
            body_ptsy[i] = -s0 * new_x + c0 * new_y;
          }


          // Calcuate the polyfit based on the x and y values
          // Eigen::VectorXd fit_coeffs;

          // Convert the std::vector to Eigen::VectorXd,
          // since this is needed by the polyfit function
          double *ptrx = &body_ptsx[0];
          double *ptry = &body_ptsy[0];
          Eigen::Map<Eigen::VectorXd> eig_ptsx(ptrx,body_ptsx.size());
          Eigen::Map<Eigen::VectorXd> eig_ptsy(ptry,body_ptsy.size());

          auto fit_coeffs = polyfit(eig_ptsx,eig_ptsy,3);

          // Since we have done the change of coordinates, cte wil lbe evaluated at x=0;
          // And for the heading error, the ego_psy value will be 0 degrees.
          // So the new state vector will be 0,0,0,v,cte,epsi

          // Calculate the cross track error
          double cte = polyeval(fit_coeffs,0.0);
          // Calculate the orientation error:
          // double ddx_fx = fit_coeffs[1] + 2*fit_coeffs[2]*px + 3*fit_coeffs[3]*px*px;
          // Since px=0, and psi=0
          double epsi = 0.0 - atan(fit_coeffs[1]);
          // std::cout<< "epsi: "<< epsi <<std::endl;

          Eigen::VectorXd current_states(6);
          current_states<< 0.0, 0.0, 0.0, v, cte, epsi;

          // Call the MPC solver.
          // vector<double> actuator_cmds;
          auto results = mpc.Solve(current_states,fit_coeffs);

          double steer_value;
          double throttle_value;

          // Reverse sign of steer_value. simulator expects reverse.
          steer_value = -1*results[0];
          throttle_value = results[1];

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          // We have the polyfit coeffs already. So lets use that to build up a reference trajectory
          double delta_dist = 3;
          int num_points = 25; // Lets see the path for 50 meters ahead of us.
          for (int i =1;i<num_points; i++){
            double curr_x = 0.0 + i*delta_dist;
            next_x_vals.push_back(curr_x);
            next_y_vals.push_back(polyeval(fit_coeffs, curr_x));

          }

          //Display the MPC predicted trajectory
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
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
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
          this_thread::sleep_for(chrono::milliseconds(100));
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

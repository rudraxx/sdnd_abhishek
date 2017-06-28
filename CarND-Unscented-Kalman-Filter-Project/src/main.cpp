#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"
using namespace std;

#include <fstream>

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

std::ofstream myfile;

int main(int argc, char** argv)
{
  uWS::Hub h;

  // Create a Kalman Filter instance
  UKF ukf;

  // using input arguments for tuning the std_a_
  // if no args  given, then the estimator will initialize with default values
  // that are specified in ukf.cpp
  std::string temp_acc = "none";
  std::string temp_ydd = "none";
  if (argc==3){
    for (int i=0;i<argc;i++){
      std::cout<< "arg["<<i<<"]= "<< argv[i] <<std::endl;
    }
    // set the ukf.std_a_ = first argument. Do this after ukf has been instantiated
    ukf.std_a_     = stod(argv[1]);
    ukf.std_yawdd_ = stod(argv[2]);

    // Create a text file to save the data
    temp_acc = argv[1];
    temp_ydd = argv[2];

  }else{
    // Create a text file to save the data
    std::cout<< "No values for sig_acc and sig_yawdd specified."<<
     "Initializing UKF with standard values." <<std::endl;
  }

  std::string filename = "log_std_acc_" + temp_acc+ "_std_ydd_" + temp_ydd+".txt";
  myfile.open(filename.c_str());//("test_data_" ); //

  // Set up the first line of log file.
  if(myfile.is_open() ){
    myfile <<"timestamp sensor sensor_x sensor_y ukf_x ukf_y x_gt y_gt RMSE(0) RMSE(1) RMSE(2) RMSE(3) nis_value" << std::endl;
    // << p_x << " " << p_y<< " "<<v1 << " "<<v2<<" "
    // << x_gt << " "<<y_gt <<" "<< vx_gt << " "<<vy_gt
    // << RMSE(0) << " "<<RMSE(1) <<" "<< RMSE(2) << " "<<RMSE(3)
    // <<std::endl;
  }


  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&ukf,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {

        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          string sensor_measurment = j[1]["sensor_measurement"];

          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
    	  long long timestamp;

    	  // reads first element from the current line
    	  string sensor_type;
    	  iss >> sensor_type;

    	  if (sensor_type.compare("L") == 0) {
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;
          		meas_package.raw_measurements_ = VectorXd(2);
          		float px;
      	  		float py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
              // log data
              if (myfile.is_open()){
                myfile <<timestamp<<" " << "L" << " "<<
                px<<" "<<py<<" ";

              }

          } else if (sensor_type.compare("R") == 0) {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		float ro;
      	  		float theta;
      	  		float ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
              // log radar data
              if (myfile.is_open()){
                myfile <<timestamp<<" " << "R" << " "<<
                ro*cos(theta)<<" "<<ro*sin(theta)<<" ";

              }
          }
          float x_gt;
    	  float y_gt;
    	  float vx_gt;
    	  float vy_gt;
    	  iss >> x_gt;
    	  iss >> y_gt;
    	  iss >> vx_gt;
    	  iss >> vy_gt;
    	  VectorXd gt_values(4);
    	  gt_values(0) = x_gt;
    	  gt_values(1) = y_gt;
    	  gt_values(2) = vx_gt;
    	  gt_values(3) = vy_gt;
    	  ground_truth.push_back(gt_values);

          //Call ProcessMeasurment(meas_package) for Kalman filter
    	  ukf.ProcessMeasurement(meas_package);

    	  //Push the current estimated x,y positon from the Kalman filter's state vector

    	  VectorXd estimate(4);

    	  double p_x = ukf.x_(0);
    	  double p_y = ukf.x_(1);
    	  double v  = ukf.x_(2);
    	  double yaw = ukf.x_(3);

    	  double v1 = cos(yaw)*v;
    	  double v2 = sin(yaw)*v;

    	  estimate(0) = p_x;
    	  estimate(1) = p_y;
    	  estimate(2) = v1;
    	  estimate(3) = v2;

    	  estimations.push_back(estimate);

    	  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
        //VectorXd RMSE = VectorXd::Zero(4);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // save data to logfile
          if (myfile.is_open()){
            myfile << p_x << " " << p_y<< " "
            << x_gt << " "<<y_gt <<" "
            << RMSE(0) << " "<<RMSE(1) <<" "<< RMSE(2) << " "<<RMSE(3)<<" "<< ukf.nis_value
            <<std::endl;
          }

        }
      } else {

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

  // close the logfile
  myfile.close();

}

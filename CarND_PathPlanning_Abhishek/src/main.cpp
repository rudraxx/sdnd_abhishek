#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "spline.h"

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
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y,const vector<double> &maps_dx,const vector<double> &maps_dy)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


vector<vector<double> > generate_traj(int horizon,double car_s, double car_d, double ref_vel, double car_x,double car_y,double car_yaw,double lane, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,const vector<double> &previous_path_x,const vector<double> &previous_path_y, const vector<double> &map_waypoints_dx, const vector<double> &map_waypoints_dy){

  int prev_size = previous_path_x.size();

  // instead of taking all points, lets take only 25 points from the last rajectory.
  // int const_num_points = 25;
  // if (const_num_points<prev_size){
  //   prev_size = const_num_points;
  // }

  // // avoid error for splines
  // if(ref_vel<0.01){
  //   ref_vel=0.1;
  // }


  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // cout<< "in gen_traj lane = "<< lane << endl;

  if(prev_size<2){
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }else{

    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];

    ref_yaw = atan2( (ref_y - ref_y_prev),(ref_x - ref_x_prev) );

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

  }

  // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
  // Find the next 5 points 30 meters apart in the frenet coordinates. 

  for (int i = 0;i<3;++i){
    vector<double> newwaypt = getXY(car_s+40*(i+1),(2+lane*4),map_waypoints_s,map_waypoints_x, map_waypoints_y,map_waypoints_dx,map_waypoints_dy);

     ptsx.push_back(newwaypt[0]);
     ptsy.push_back(newwaypt[1]);

  }

  // All the ptsx and ptsy points are in the world reference frame. Converting those to body coordinates.
  for (int i = 0;i<ptsx.size(); ++i){
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0-ref_yaw));
  }

  // calculate the spline values
  tk::spline s;

  // print ptsx data
/*            std::cout<< "Printing ptsx and ptsy... " <<std::endl;
  for (int j=0;j<ptsx.size();++j){
    std::cout<< "pt: "<<j<< " x: "<<ptsx[j] << " y: "<< ptsy[j]<<std::endl; 
  }
*/
  s.set_points(ptsx,ptsy);
  
  vector<double> next_x_vals;
  vector<double> next_y_vals;


  for(int i=0;i<prev_size;++i){
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  // calculate how to break up the spline data to get x,y pts.
  double target_x = 50.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for (int i=0;i<=horizon-prev_size; ++i){
    
    double N = (target_dist/(0.02*ref_vel/2.24));
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to world coordinate system
    x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
    y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  // get s ,d data for all the traj points.
  vector<double> next_s_data;
  vector<double> next_d_data;
  vector<double> sd_data_point;
  
  for (int i=0; i< next_x_vals.size(); ++i){

    if (i<next_x_vals.size()-1){

      double px1 = next_x_vals[i];
      double py1 = next_y_vals[i];

      double px2 = next_x_vals[i+1];
      double py2 = next_y_vals[i+1];

      double angle = atan2(py2-py1, px2-px1) ;

      sd_data_point = getFrenet( px1,  py1, angle, map_waypoints_x, map_waypoints_y);

      next_s_data.push_back(sd_data_point[0]);
      next_d_data.push_back(sd_data_point[1]);
      // cout<< "xval: " << px1 << " yval: " << py1 << " s val = "<<sd_data_point[0] << " d_val="<< sd_data_point[1]<<endl;

  }else{
          // use previous value.
      next_s_data.push_back(sd_data_point[0]);
      next_d_data.push_back(sd_data_point[1]);
  }

}
  

  return {next_x_vals,next_y_vals,next_s_data,next_d_data};
}





int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  double ref_vel = 1;//10*2.24; // mph 
  int horizon = 50;
  double timestep = 0.02;

  // std::string current_state = "KL";
  std::string target_state = "KL";
  int target_lane = 1;

  double target_velocity = 40;


  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&target_state, &map_waypoints_dx,&map_waypoints_dy, &target_velocity,&timestep,&horizon, &target_lane,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

//**** Obstacle calculations:
            // create vector for all storing array_sd data for all objects. Storing data in frenet coordinates
            vector<vector<vector<double> > > obj_traj;
            cout<< "\n ************\n Step 1: Obstacle trajectories..." << std::endl;

            for (int i=0;i<sensor_fusion.size();++i){
              // generate the trajectory for next horizon steps for each object.
              // car in my lane?
              double px = sensor_fusion[i][1];
              double py = sensor_fusion[i][2];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_car_s = sensor_fusion[i][5];
              double d = sensor_fusion[i][6];

              double obj_speed = sqrt(vx*vx + vy*vy);
              double obj_theta = atan2(vy,vx);
              //cout<< "Speed: object["<<i<<"] = "<< obj_speed<<endl;

              // create vector for s & d values for the objects
              vector<double> sddata;
              // create a vector for storing the s&d values for next "horizon" timesteps
              vector<vector<double> > array_sd;
              for (int j=0;j<horizon; ++j){
                // Calculate next x and y in global coordinates
                double new_s = check_car_s + (j)*obj_speed*timestep;
                // consider objects maintain their lanes.
                double new_d = d;

                sddata = {new_s,new_d};

                array_sd.push_back(sddata);
              }
              // Push this packet onto the obj_traj vector
              obj_traj.push_back(array_sd);
            }


            // Creation of spline for vehicle motion start coding from here.
//***************

            std::vector<string> fsm_states = {"KL","LCL","LCR"};
            // std::map<std::string, vector<string> > fsm; 
            // fsm["KL"] = {"KL,","LCL","LCR"};
            // fsm["LCL"] = {"LCL","KL"};
            // fsm["LCR"] = {"LCR","KL"};

            // Calculate the cost for the next set of potential states based on the current state;
            //vector<string> potential_states = fsm[current_state];
            vector<double> fsm_costs;
            vector< vector<vector<double> > > next_array;

            vector<double> test_next_x_vals;
            vector<double> test_next_y_vals; 
            vector<vector<double> > test_next_xy_vals;
            vector<double> test_next_s_vals;
            vector<double> test_next_d_vals;

            vector<int> target_lane_array;


            vector<double> next_x_vals;
            vector<double> next_y_vals;
            vector<double> next_s_vals;
            vector<double> next_d_vals;


            bool too_close = false;
            bool too_far_left = false;
            bool too_far_right = false;
            int temp_lane= target_lane;

            std::map<std::string,std::vector<string> > fsm_map;
            fsm_map["KL"] = {"KL","PLCL","PLCR"};
            fsm_map["PLCL"] = {"PLCL","KL","LCL"};
            fsm_map["PLCR"] = {"PLCR","KL","LCR"};

            fsm_map["LCL"] = {"LCL","KL"};
            fsm_map["LCR"] = {"LCR","KL"};

            // cout<< "FSM Map: \n";
            // vector<string> mystr = fsm_map["KL"];
            // for (int kk=0;kk<mystr.size(); ++kk){
            //   cout<< "fsm_map[KL]: "<< mystr[kk] << endl;
            // }


            //cout<< "Step 3: FSM evaluation..." << std::endl;

            // Check if the maneuver is complete. If not, stay in the old state
            bool maneuver_complete = false;

            if( fabs(car_d - (2+4*target_lane)) < 0.5){
              maneuver_complete = true;
              // current_state = "KL";
            }

            if ( maneuver_complete==true){

              // Do the steps for KL state:
              for (int i=0;i<fsm_states.size(); ++i) {
              // for (int i=0;i<1; ++i){
                
                //cout<< "Current ego states:\n";
                //cout<< "car_x: "<<car_x<<"car_y: "<<car_y<<"car_yaw: "<<car_yaw<< std::endl;              
                // execute the potential state:
                if (i==0){
                  cout<< "FSM Case 0... KL"<<endl;
                  // cout<< "ref vel: " << ref_vel << endl;
                  temp_lane = target_lane + 0;
                  // cout<< "Evaluating for lane no. : "<< temp_lane << std::endl;
                  // next_xy_vals = generate_ego_trajectory(ego,horizon,temp_lane,map_waypoints_s,map_waypoints_x,map_waypoints_y,previous_path_x,previous_path_y,map_waypoints_dx,map_waypoints_dy);
            
                }
                else if (i==1){ 
                  cout<< "FSM Case 1...LCL"<<endl ;
                  // cout<< "ref vel: " << ref_vel << endl;
                  temp_lane = target_lane - 1;
                  // cout<< "Evaluating for lane no. : "<< temp_lane << std::endl;
                  // next_xy_vals = generate_ego_trajectory(ego,horizon,temp_lane,map_waypoints_s,map_waypoints_x,map_waypoints_y,previous_path_x,previous_path_y,map_waypoints_dx,map_waypoints_dy);
                }
                else{
                  cout<< "FSM Case 2...LCR" <<endl;
                  // cout<< "ref vel: " << ref_vel << endl;
                  temp_lane = target_lane + 1;
                  // cout<< "Evaluating for lane no. : "<< temp_lane << std::endl;
                  // next_xy_vals = generate_ego_trajectory(ego,horizon,temp_lane,map_waypoints_s,map_waypoints_x,map_waypoints_y,previous_path_x,previous_path_y,map_waypoints_dx,map_waypoints_dy);
                }

                test_next_xy_vals = generate_traj(horizon,car_s,car_d,ref_vel,car_x,car_y,car_yaw,temp_lane,map_waypoints_s, map_waypoints_x, map_waypoints_y, previous_path_x,previous_path_y,map_waypoints_dx, map_waypoints_dy);

                // Get the new calculated values for x,y,s,d trajectories              
                test_next_x_vals = test_next_xy_vals[0];
                test_next_y_vals = test_next_xy_vals[1];
                test_next_s_vals = test_next_xy_vals[2];
                test_next_d_vals = test_next_xy_vals[3];

                // Assign all trajectories to a vector. Will be used later for choosing the lowest cost state 
                next_array.push_back(test_next_xy_vals);

                // Assign the targe lane for each state  
                target_lane_array.push_back(temp_lane);


  // *** Cost calculations:
                // cout<< "Calculate costs..." << "Number of obstacles: "<<obj_traj.size()<< std::endl;

                double cost = 0.0;

                // calculate costs of hitting another vehicle
                for (int j=0; j<obj_traj.size(); ++j){
                  // Test if the ego vehicle trajectory collides with any of the obstacles
                  vector<vector<double> > current_obstacle = obj_traj[j];
                  for(int k=1;k<current_obstacle.size() ; ++k){
                    // cout<< "current_obstacle[k][0] : "<<current_obstacle[k][0]<<", current_obstacle[k][1] : "<<current_obstacle[k][1];
                    // cout<< ", next_x_vals[k] : "<< next_x_vals[k] << ", next_y_vals[k] : " << next_y_vals[k]<< std::endl;

                    // double temp_theta =  atan2(next_y_vals[k]-next_y_vals[k-1], next_x_vals[k]-next_x_vals[k-1]);
                    // vector<double> temp_sd_vals = getFrenet(next_x_vals[k],next_y_vals[k], temp_theta, map_waypoints_x,map_waypoints_y);
                    // vector<double> temp_sd_vals = getFrenet(next_x_vals[k],next_y_vals[k], temp_theta, map_waypoints_x,map_waypoints_y);

                    // obstacle  in my lane?
                    if(current_obstacle[k][1]> (2+4*temp_lane - 2) && current_obstacle[k][1] < (2 + 4*temp_lane + 2) ){
                      // distance less than some threshold;  
                      if ( (current_obstacle[k][0]>test_next_s_vals[k]) && ( (current_obstacle[k][0]-test_next_s_vals[k]) < 30 ) ) {

                    // if ( distance(current_obstacle[k][0],current_obstacle[k][1],next_x_vals[k],next_y_vals[k]) < 1){
                        cost += 10; // cost of collision
                        //cout<< "fsm: "<< i<<", Collision at timestep k = "<<k << " between ego and vehicle id: "<<j<<std::endl;
                        // set the too close flag to true. THis will be used to decelerate the vehicle.  
                          if(i==0){
                            too_close = true;
                          }
                      } // distance loop
                    } // in lane loop
                  } // each timestep in traj
                } //each obstacle

                // cost of going off track
                // cout<< "Calculate cost of going off track..."<<endl;

                if( (2+ 4*temp_lane) >12 ){
                  cost += 10000;
                  cout<< "Far right. take action.."<< endl;
                  too_far_right = true;
                } 

                if( (2 + 4*temp_lane) < 0 ){
                  cost += 10000;
                  cout<< "Far left. take action.."<< endl;
                  too_far_left = true;
                } 


                // for (int m=1; m< test_next_x_vals.size(); ++m){
                //   // cout<< "s val = "<<test_next_s_vals[0] << "d_val="<< test_next_d_vals[0]<<endl;

                //   if(test_next_d_vals[m] <1 || test_next_d_vals[m] > 11){
                //     // cout<< "Off track..."<<endl;
                //     cost += 10000;

                //     if(test_next_d_vals[m] <1){
                //       cout<< "Far left. take action.."<< endl;
                //       too_far_left = true;
                //     }

                //     if(test_next_d_vals[m] >11){
                //       cout<< "Far right. take action.."<< endl;
                //       too_far_right = true;
                //     }

                //   }
                // }
                // cout<< "After off track: "<< cost<< endl; 

                // In general, prefer middle lane instead of changing lane, when cost for all 3 is the same.
                if (temp_lane !=1){
                  // Add nominal cost ~ 10 when we are considering any state other than KL.
                  cost += 3;
                }

                // cost of overtaking when there is another vehicle in the lane
                // FSM 1 : LCL ;  FSM 2: LCR

                if (i==1 || i==2){
                  for (int j=0; j<obj_traj.size(); ++j){
                    // Test if there is another vehicle in the adjacent lane
                    vector<vector<double> > current_obstacle = obj_traj[j];
                    for(int k=1;k<current_obstacle.size() ; ++k){

                      // obstacle s within range:   
                      if ( fabs(current_obstacle[k][0]- test_next_s_vals[k]) < 20) {
                        if (i==1){
                          // For left lane change, check if there is a car in the left lane:
                          if ( ( test_next_d_vals[k]-current_obstacle[k][1] < 5) && (test_next_d_vals[k]-current_obstacle[k][1] > 2) ){
                            cost+= 20;
                          } 

                        } 
                        else{
                          // i==2
                          if ( ( current_obstacle[k][1] - test_next_d_vals[k] < 5) && (current_obstacle[k][1] - test_next_d_vals[k] > 2) ) {
                            cost+= 20;
                          } 
                        }
                      } // obs within range loop
                    } // each timestep in traj
                  } // num obstacles
                } // fsm 1 or 2



                cout<< "\n cost for state "<< i<< " is : " << cost <<"\n"<< std::endl;


                fsm_costs.push_back(cost);

              } // fsm_states

  // Best state selection

              cout<< "Searching for best state..." << std::endl;

              // // double min_cost = 9999999;
              int min_cost_index = 0;

              // // Find index of the smallest element
              auto ptr_min_cost = std::min_element(std::begin(fsm_costs),std::end(fsm_costs));
              min_cost_index = std::distance(fsm_costs.begin(),ptr_min_cost);


              // vector<double> next_x_vals;
              // vector<double> next_y_vals;
              // vector<double> next_s_vals;
              // vector<double> next_d_vals;
              // int target_lane;

              next_x_vals = next_array[min_cost_index][0];
              next_y_vals = next_array[min_cost_index][1];
              next_s_vals = next_array[min_cost_index][2];
              next_d_vals = next_array[min_cost_index][3];

              // Get the target lane
              target_lane = target_lane_array[min_cost_index];

              target_state = fsm_states[min_cost_index];
              cout<< "Stable.. Target state: " << target_state  << ", Target lane: " << target_lane << "\n"<< endl;

            } 

            else{

              // stay in the previous state.
              // calculate trajectory based on the old lane.
              test_next_xy_vals = generate_traj(horizon, car_s,car_d,ref_vel,car_x,car_y,car_yaw,target_lane,map_waypoints_s, map_waypoints_x, map_waypoints_y, previous_path_x,previous_path_y,map_waypoints_dx, map_waypoints_dy);


              // Get the new calculated values for x,y,s,d trajectories              
              next_x_vals = test_next_xy_vals[0];
              next_y_vals = test_next_xy_vals[1];
              next_s_vals = test_next_xy_vals[2];
              next_d_vals = test_next_xy_vals[3];
              cout<< "In transition.. Target state: " << target_state  << ", Target lane: " << target_lane << "\n"<< endl;
            }




            // // get data from the fsm1 array - Temporary, while cost is not being calculated.
            // next_x_vals = next_array[0][0];
            // next_y_vals = next_array[0][1];
            // next_s_vals = next_array[0][2];
            // next_d_vals = next_array[0][3];

// Controls
            // Update the commands based on the selected state:
            // For KL state, see if we should accelerate or decelerate
            if (target_state == "KL"){

              if (too_close == true){
                ref_vel -= 0.224;
                cout<< "Decelerating..."<< "ref_vel = " << ref_vel<<"\n"<< endl;
              }
              else if (car_speed < target_velocity){
                ref_vel += 0.224;
                cout<< "Accelerating..."<< "ref_vel = " << ref_vel<<"\n"<< endl;
              }
            }
            // if( (min_cost_index == 1) || (too_far_right==true) ){
            //   lane = target_lane;
            // }
            // if( (min_cost_index == 2) || (too_far_left==true) ){
            //   lane = target_lane;
            // }


// **********************************8


          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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

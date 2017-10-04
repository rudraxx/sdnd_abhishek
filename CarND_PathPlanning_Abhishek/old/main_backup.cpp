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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

template <class T> 
//void display_vector(vector<double> data){
void display_vector(vector<T> data){

    std::cout<< "Data size: "<< data.size() <<  std::endl;
    for (int i=0;i<data.size();++i){
      if(i==0){
        std::cout<<data[i];
      }else{
        std::cout<< " , " << data[i];
      }
    }

    std::cout<< "\n"<<std::endl;    
  
}

vector<vector<double> > generate_ego_trajectory(double car_d,double car_s,double ref_vel,int horizon,double car_x, double car_y,double car_yaw,int lane, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,const vector<double> &previous_path_x,const vector<double> &previous_path_y){

  int prev_size = previous_path_x.size();

  // instead of taking all points, lets take only 25 points from the last rajectory.
  int const_num_points = 40;
  if (const_num_points<prev_size){
    prev_size = const_num_points;
  }

  // Creation of spline for vehicle motion start coding from here.
  // vector<double> ptsx;
  // vector<double> ptsy;

  vector<double> pts_s;
  vector<double> pts_d;



  // double ref_x = car_x;
  // double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // If we dont have a lot of points from the previous trajectory, 
  // that means that the car is at the end of the previous trajectory.
  // The simulator sends all unused points.  

  double ref_s = car_s;
  double ref_d = car_d;

  double car_s_prev;
  double car_d_prev;

  if(prev_size<2){
    cout<<"in prev_size< 2 loop"<< endl;
    // double prev_car_x = car_x - cos(ref_yaw); // small delta  back from current point
    // double prev_car_y = car_y - sin(ref_yaw); 

    // ptsx.push_back(prev_car_x);
    // ptsx.push_back(car_x);

    // ptsy.push_back(prev_car_y);
    // ptsy.push_back(car_y);
    car_s_prev = car_s - 0.02* ref_vel/2.24;
    car_d_prev = car_d;

    pts_s.push_back(car_s_prev);
    pts_s.push_back(ref_s);

    pts_d.push_back(car_d_prev);
    pts_d.push_back(ref_d);

  }else{

    // Use the last 2 points on the previous_path to calculate x,y and heading angle theta. 
    // Use these two points to the starting points for the next set of anchorpoints.
    // Since we have 
    // lot of points on the previous_path_* , we will just add more points after the last point  
    // of this data . This will enable smooth transitions.  
    // cout<<"\n in prev_size> 2 loop. Prev_size = "<< prev_size<<endl;
    // cout<< "ref_vel= "<<ref_vel<<endl;

    double ref_x = previous_path_x[prev_size-1];
    double ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2]; // -1 is fudge factor.
    double ref_y_prev = previous_path_y[prev_size-2];

    double ref_x2_prev = previous_path_x[prev_size-3];
    double ref_y2_prev = previous_path_y[prev_size-3];


    ref_yaw  = atan2( (ref_y - ref_y_prev),(ref_x - ref_x_prev) );
    double ref_yaw2 = atan2( (ref_y_prev - ref_y2_prev),(ref_x_prev - ref_x2_prev) );

    // cout<< "ref_x: "<<ref_x<<" ref_y: " <<ref_y << "ref_yaw : "<<ref_yaw << std::endl;
    // cout<< "ref_x_prev: "<<ref_x_prev<<" ref_y_prev: " <<ref_y_prev << "ref_yaw : "<<ref_yaw2 << std::endl;
    // cout<< "ref_x2_prev: "<<ref_x2_prev<<" ref_y2_prev: " <<ref_y2_prev << std::endl;

    vector<double> temp_sd_data;
    // calculate the sd data for last point on the previous trajectory
    temp_sd_data = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x,map_waypoints_y);
    ref_s = temp_sd_data[0];
    ref_d = temp_sd_data[1];

    // calculate the sd data for second last point on the previous trajectory
    temp_sd_data = getFrenet(ref_x_prev, ref_y_prev, ref_yaw2, map_waypoints_x,map_waypoints_y);

    car_s_prev = temp_sd_data[0];
    car_d_prev = temp_sd_data[1];

    // NEed to ensure prev value is not greater than ref_s
    if (car_s_prev>=ref_s){
      car_s_prev = ref_s - 0.1;
    }

    pts_s.push_back(car_s_prev);
    pts_s.push_back(ref_s);

    pts_d.push_back(car_d_prev);
    pts_d.push_back(ref_d);

  }

  // #Get the frenet coordinates of the 25th point. (const_num_points). 
  // This will be used to select 3 more points in front of that trajectory.
  // vector<double> temp_sd = getFrenet(ref_x,ref_y, ref_yaw, map_waypoints_x,map_waypoints_y);

  // cout<< "Printing in generate_ego_trajectory loop :   ";
  // cout<< "Lane: " << lane<<"\n"<< endl ;

  // take 3 waypts from behind and 3 from front of the ref_x and ref_y values. This will help with a smooth curve.
  for (int i = 0;i<3;++i){
    vector<double> newwaypt = {ref_s+30*(i+1),(double)(2+lane*4)};

//    newwaypt = getXY(temp_sd[0]+30*(i+1),(2+lane*4),map_waypoints_s,map_waypoints_x, map_waypoints_y);
    // vector<double> newwaypt = getXY(car_s+30*(i+1),(2+lane*4),map_waypoints_s,map_waypoints_x, map_waypoints_y);

    pts_s.push_back(newwaypt[0]);
    pts_d.push_back(newwaypt[1]);

    // cout<< "idx: " << i<< ", newwaypt[0]: "<<newwaypt[0]<<" , newwaypt[1]: "<<newwaypt[1]<<std::endl;
  }

  // for(int i=0; i< pts_s.size(); ++i){
  //   cout<< "idx: " << i<< ", pts_s[i]: "<<pts_s[i]<<" , pts_d[i]: "<<pts_d[i]<<std::endl;
  // }


  // // All the ptsx and ptsy points are in the world reference frame. Converting those to body coordinates.
  // for (int i = 0;i<ptsx.size(); ++i){
  //   double shift_x = ptsx[i] - ref_x;
  //   double shift_y = ptsy[i] - ref_y;

  //   ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0-ref_yaw));
  //   ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0-ref_yaw));
  // }

  // calculate the spline values
  // Using 
  tk::spline s;

  // print ptsx data
/*            std::cout<< "Printing ptsx and ptsy... " <<std::endl;
  for (int j=0;j<ptsx.size();++j){
    std::cout<< "pt: "<<j<< " x: "<<ptsx[j] << " y: "<< ptsy[j]<<std::endl; 
  }
*/
  s.set_points(pts_s,pts_d);
  
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  vector<double> next_s_vals;
  vector<double> next_d_vals;


  // Add the first 20 points from previous_path_x&y, so that we have some continuity.
  for(int i=0;i<prev_size;++i){
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
    // in order to get the s, d values of the previous path, lets fit a spline to the data

    double angle = atan2(previous_path_y[i+1]-previous_path_y[i], previous_path_x[i+1]-previous_path_x[i]);

    // calculate the new frenet coordinates
    vector<double> temp_sddata = getFrenet(previous_path_x[i],previous_path_y[i], angle, map_waypoints_x,map_waypoints_y);

    // also push back the s and d values
    next_s_vals.push_back(temp_sddata[0]);
    next_d_vals.push_back(temp_sddata[1]);

  }

  // calculate how to break up the spline data to get x,y pts.
  double target_s = 50.0;
  double target_d = s(target_s);
  // double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double new_s_point;
  double new_d_point;
  double s_temp;

  // set s_temp to be the last s value for the previous traj
  s_temp = ref_s;

  for (int i=0;i<=(horizon-prev_size); ++i){
    s_temp += 0.02*ref_vel/2.24;

    new_s_point = s_temp;
    new_d_point = s(new_s_point);

    vector<double> newwaypt = getXY(new_s_point,new_d_point,map_waypoints_s,map_waypoints_x, map_waypoints_y);

    next_x_vals.push_back(newwaypt[0]);
    next_y_vals.push_back(newwaypt[1]);

    next_s_vals.push_back(new_s_point);
    next_d_vals.push_back(new_d_point);
  }
  // cout<< "last recorded new_s_point: "<< s_temp<<"d point: "<<new_d_point<< endl;


  // // Printing values from the final trajectory
  // for (int i=0; i<6;++i){
  //   cout<< "next_x_vals["<< i*10<< "] = "<< next_x_vals[i*10]<<" " ;
  //   cout<< "next_y_vals["<< i*10<< "] = "<< next_y_vals[i*10]<< endl;
  // }

  return {next_x_vals,next_y_vals,next_s_vals,next_d_vals};
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  ofstream myfile;
  myfile.open("test_data.txt");
  myfile<< "epoch horizon s1 d1 s2 d2 s3 d3 s4 d4 s5 d5 s6 d6 s7 d7 s8 d8 s9 d9 s10 d10 s11 d11 s12 d12 fsm1_x fsm1_y fsm2_x fsm2_y fsm3_x fsm3_y  \n";

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  long long int epoch = 0;

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
  int lane = 1;
  double ref_vel = 1; // mph 
  int horizon = 50;
  double timestep = 0.02;
  std::string current_state = "KL";
  double target_velocity = 45;

  h.onMessage([&epoch,&myfile,&target_velocity,&current_state,&timestep,&horizon,&lane,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

            // abhishek code start:

            // Print the previous_path_x info, so that we can understand it better.
            // std::cout<< "Printing data for previous_path_x..."<<std::endl;
            // display_vector<double>(previous_path_x);

            // Step 1: For all the target objects, generate the potential trajectories for next 50 timesteps.
            // Step 2: For the candidate FSMs, calculate the trajectories. 
            // Step 3: Based on the predictions and ego behavior, evaluate total cost of each of the maneuvers.
            // Step 4: Choose the maneuver which has the least cost.
            // Step 5: Based on this decision, determine what the ego vehicle should do. This will be the action for the given state.
                        // eg. accelerate, decelerate, turn left, turn right etc. 


            // // Logic for preventing collision
            // if (prev_size>0){
            //   car_s = end_path_s;
            // }
            // bool too_close = false;
            // find ref_v to use

            // Implement Step 1: For all the target objects, generate the potential trajectories based on data...

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

            // Implement Step 2: Generate current trajectory of ego current path
            // So now, we have athe trajectories of all objects in sd coordinates for the next "horizon" timesteps.


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            // Find the next 3 points 30 meters apart in the frenet coordinates. 

            // The ref_x, ref_y,ref_yaw are the starting points from which the next addition of points will happen.
            // I think it makes sense to use the s,d coordinates of that point to calculate the next 3 major anchorpoints,
            // instead of the current car_s
            // vector<vector<double> > next_xy_vals;
            // next_xy_vals = generate_ego_trajectory(ref_vel,horizon,car_x,car_y,car_yaw,lane,map_waypoints_s,map_waypoints_x,map_waypoints_y,previous_path_x,previous_path_y);

            // vector<double> next_x_vals = next_xy_vals[0];
            // vector<double> next_y_vals = next_xy_vals[1];

            // Implement Step 3: 
            // Now we have the object trajectories and the current ego trajectory.
            // Step 3: Based on the predictions and ego behavior, evaluate various next states

            std::vector<string> fsm_states = {"KL","LCL","LCR"};
            // std::map<std::string, vector<string> > fsm; 
            // fsm["KL"] = {"KL,","LCL","LCR"};
            // fsm["LCL"] = {"LCL","KL"};
            // fsm["LCR"] = {"LCR","KL"};

            // Calculate the cost for the next set of potential states based on the current state;
            //vector<string> potential_states = fsm[current_state];
            vector<double> fsm_costs;
            vector< vector<vector<double> > > next_array;

            vector<double> next_x_vals;
            vector<double> next_y_vals; 
            vector<vector<double> > next_xy_vals;
            vector<double>next_s_vals;
            vector<double>next_d_vals;

            bool too_close = false;
            int temp_lane= lane;

            //cout<< "Step 3: FSM evaluation..." << std::endl;

            for (int i=0;i<fsm_states.size(); ++i){
              
              //cout<< "Current ego states:\n";
              //cout<< "car_x: "<<car_x<<"car_y: "<<car_y<<"car_yaw: "<<car_yaw<< std::endl;              
              // execute the potential state:
              if (i==0){
                // cout<< "FSM Case 0...";
                // cout<< "ref vel: " << ref_vel << endl;
                temp_lane = lane + 0;
                // cout<< "Evaluating for lane no. : "<< temp_lane << std::endl;
                next_xy_vals = generate_ego_trajectory(car_d,car_s,ref_vel,horizon,car_x,car_y,car_yaw,temp_lane,map_waypoints_s,map_waypoints_x,map_waypoints_y,previous_path_x,previous_path_y);
              }
              else if (i==1){ 
                // cout<< "FSM Case 1..." ;
                // cout<< "ref vel: " << ref_vel << endl;
                temp_lane = lane - 1;
                // cout<< "Evaluating for lane no. : "<< temp_lane << std::endl;

                next_xy_vals = generate_ego_trajectory(car_d,car_s,ref_vel,horizon,car_x,car_y,car_yaw,temp_lane,map_waypoints_s,map_waypoints_x,map_waypoints_y,previous_path_x,previous_path_y);
              }
              else{
                // cout<< "FSM Case 2..." ;
                // cout<< "ref vel: " << ref_vel << endl;
                temp_lane = lane + 1;
                // cout<< "Evaluating for lane no. : "<< temp_lane << std::endl;
                next_xy_vals = generate_ego_trajectory(car_d,car_s,ref_vel,horizon,car_x,car_y,car_yaw,temp_lane,map_waypoints_s,map_waypoints_x,map_waypoints_y,previous_path_x,previous_path_y);
              }
              next_x_vals = next_xy_vals[0];
              next_y_vals = next_xy_vals[1];
              next_s_vals = next_xy_vals[2];
              next_d_vals = next_xy_vals[3];
              next_array.push_back(next_xy_vals);  

              // cout<< "Calculate costs..." << "Number of obstacles: "<<obj_traj.size()<< std::endl;

              // calculate costs of each trajectory

              double cost = 0.0;
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
                    if ( (current_obstacle[k][0]>next_s_vals[k]) && ( (current_obstacle[k][0]-next_s_vals[k]) < 40 ) ) {

                  // if ( distance(current_obstacle[k][0],current_obstacle[k][1],next_x_vals[k],next_y_vals[k]) < 1){
                      cost += 100; // cost of collision
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
              for (int m=1; m< next_x_vals.size(); ++m){
                // cout<< "s val = "<<sd_vals[0] << "d_val="<< sd_vals[1]<<endl;

                if(next_d_vals[m] <0 || next_d_vals[m] > 12){
                  // cout<< "Off track..."<<endl;
                  cost += 1000;
                }
              }

              // In general, prefer middle lane instead of changing lane, when cost for all 3 is the same.
              if (temp_lane !=1){
                // Add nominal cost ~ 10 when we are considering any state other than KL.
                cost += 3;
              }
              cout<< "cost for state "<< i<< " is : " << cost <<"\n"<< std::endl;


              fsm_costs.push_back(cost);     

            }// each fsm state

            //log data
                        // log data
            // cout<< "Logging data..."<<endl;
            for (int j=0; j<horizon;++j){
              myfile<< epoch<<" "; // s data
              myfile<< j<<" "; // s data
              
              // cout<< "Logging sensor fusion data..."<<endl;
              for (int i=0;i<sensor_fusion.size();++i){
                myfile<< obj_traj[i][j][0]; // s data
                myfile<< " " ;
                myfile<< obj_traj[i][j][1]; // d data
                myfile<< " " ;
              }
              // cout<< "Logging ego fsm trajectory data ..."<<endl;
              // myfile<< "\n";
              for (int tt=0;tt<3;++tt){
                myfile<< next_array[tt][0][j]<< " ";
                myfile<< next_array[tt][1][j]<< " ";
              }
              myfile<< "\r\n";
            }
            // update epoch number
            epoch += 1;



            // cout<< "Searching for best state..." << std::endl;


            // double min_cost = 9999999;
            int min_cost_index = 0;

            // Find index of the smallest element
            auto ptr_min_cost = std::min_element(std::begin(fsm_costs),std::end(fsm_costs));
            min_cost_index = std::distance(fsm_costs.begin(),ptr_min_cost);

            string state = fsm_states[min_cost_index] ;
            cout<< "chosen state: " <<state  <<"\n"<< endl;
            next_x_vals = next_array[min_cost_index][0];
            next_y_vals = next_array[min_cost_index][1];
            next_s_vals = next_array[min_cost_index][2];
            next_d_vals = next_array[min_cost_index][3];

            // get data from the fsm1 array
            // next_x_vals = next_array[0][0];
            // next_y_vals = next_array[0][1];
            // vector<double>next_s_vals = next_array[0][2];
            // vector<double>next_d_vals = next_array[0][3];

            // Update the commands based on the selected state:
            // For KL state, see if we should accelerate or decelerate
            if (min_cost_index == 0){
              if (too_close == true){
                ref_vel -= 0.224;
                cout<< "Decelerating..."<< "ref_vel = " << ref_vel<<"\n"<< endl;
              }
              else if (car_speed < target_velocity){
                ref_vel += 0.224;
                cout<< "Accelerating..."<< "ref_vel = " << ref_vel<<"\n"<< endl;
              }
            }
            if (min_cost_index == 1){
              lane -= 1;
            }
            if (min_cost_index == 2){
              lane += 1;
            }

            // Printing values from the final trajectory
            cout<< "\n Final trajectory sent to the simulator: \n";
            for (int i=0; i<11;++i){
              cout<< "next_x_vals["<< i*5<< "] = "<< next_x_vals[i*5]<< " ";
              cout<< "next_y_vals["<< i*5<< "] = "<< next_y_vals[i*5]<< " ";
              if(i!=0){              
                cout<< "dx/dt: "<<  (next_x_vals[i*5] -  next_x_vals[(i-1)*5])/(5*timestep)<<" ";
              }
               cout<< " "<< std::endl; 
            }
            cout<< " s and d values: \n";
            for (int i=0; i<11;++i){
              cout<< "next_s_vals["<< i*5<< "] = "<< next_s_vals[i*5]<< " ";
              cout<< "next_d_vals["<< i*5<< "] = "<< next_d_vals[i*5]<< " ";
              if(i!=0){              
                cout<< "ds/dt: "<<  (next_s_vals[i*5] -  next_s_vals[(i-1)*5])/(5*timestep)<<" ";
              }
               cout<< " "<< std::endl; 
            }



            // Send the values to the simulator  
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



// Extra code.


              // if(d> (2+4*lane - 2) && d< (2 + 4*lane + 2 ) ){

              //   check_car_s += ((double)prev_size*0.02*obj_speed);


              // std::cout<< "Printing data for sensor fusion item "<< i<<std::endl;
              // display_vector<double>(sensor_fusion[i]);

              // // car in my lane?
              // float d = sensor_fusion[i][6];
              // if(d> (2+4*lane - 2) && d< (2 + 4*lane + 2 ) ){
              //   double vx = sensor_fusion[i][3];
              //   double vy = sensor_fusion[i][4];
              //   double check_speed = sqrt(vx*vx + vy*vy);
              //   double check_car_s = sensor_fusion[i][5];

              //   check_car_s += ((double)prev_size*0.02*check_speed);

              //   // check s values greater than mine and s gap
              //   if( (check_car_s>car_s) && (check_car_s-car_s)<30 ){

              //     // just reducing or speed.
              //     //ref_vel = 20;
              //     too_close = true;
                // }
            //   }
            // }

            // if(too_close){
            //   if(lane>0){
            //     lane =0;
            //   }
            //   ref_vel -= 0.224;
            // }else if(ref_vel<48){
            //   ref_vel += 0.224;
            // }


#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::abs;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  int lane = 1; 			  // start in lane 1
  double ref_vel = 0;   	  // start in 0mph
  double max_ref_vel = 49.5;  // avoid going faster than 50mph
  bool changing_lane = false; // flag when car is changing lane
  int prev_lane = lane; 	  // previous lane

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,
               &max_ref_vel,&changing_lane,&prev_lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /*******************************
          // Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
		  *******************************/
          int prev_size = previous_path_x.size();
          if (prev_size > 0) {
            car_s = end_path_s;
            car_d = end_path_d;
          }
          
          //std::cout<<"lane = "<<get_lane(car_d)<<" | car_d = "<<car_d<<" | car_s"<<car_s<<"\n";
          
          // Process data from sensor fusion
          bool too_close = false;
          bool almost_close = false;
          double next_car_speed = 0.; 
          
          bool try_change_lane = false;
          vector<bool> free_lanes = {true, true, true};
          free_lanes[lane] = false;
          
          int car_lane = get_lane(car_d);
          
          //Check if changing_lane finished.
          if(changing_lane){
          	changing_lane = check_changing_lane_end(car_d,lane);            
          }

          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            int check_car_lane = get_lane(d);
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += (double)prev_size*0.02*check_speed; //if using previous points can project s value out 
			
            double car_gap = check_car_s-car_s;
            
            if(check_car_lane == lane && check_car_s>car_s){ // Car ahead
              if(car_gap<30.0){ // Too close -> slow down
              	too_close = true;
              	try_change_lane = true;
              } else if (car_gap < 35.0){ // Close -> copy velocity
                almost_close = true;
              	next_car_speed = check_speed;                
              	try_change_lane = true;
              }
            } else if(check_car_lane != lane && abs(car_gap)<15.0){
            	free_lanes[check_car_lane] = false;
            }       
          }

          //Take action
          double diff_speed = .224;
          if(too_close){ // Too close -> slow down
          	ref_vel -= diff_speed;
          } else if(almost_close){ // Close -> copy velocity
            if(!(ref_vel<next_car_speed*1.05 && ref_vel>next_car_speed*0.95)){ // Add 5% of tolerance to the speed of the vehicle ahead
              if(ref_vel<next_car_speed && ref_vel<max_ref_vel){
          		ref_vel += diff_speed;
              }
              else if(ref_vel>next_car_speed){
          		ref_vel -= diff_speed;
              }
            }
          } else if(ref_vel<max_ref_vel){ // All cleaned
              ref_vel += diff_speed;
          }
          
          if(!changing_lane){
            if(try_change_lane){
              // std::cout<<free_lanes[0]<<" "<<free_lanes[1]<<" "<<free_lanes[2]<<"\n";
              for (int i=0; i<3; i++) {
                 if (free_lanes[i] && abs(lane-i)==1) {
                   changing_lane=true;
                   prev_lane = lane;
                   lane = i;
                   break;
                 }
              }
            }
          }else{ // Changing lane
            if(!free_lanes[lane]){ //If lane is not free, abort and come back to current lane
              // changing_lane=true; -> redundant
              lane = prev_lane;
            }
          }

          vector<double> ptsx;
          vector<double> ptsy;

          // Reference x, y, yaw states
          // Either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If previous size is almost empty, use the car as starting reference
          if(prev_size < 2){
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          } else{
              // Use the previous path's end point as starting reference
              // Redefine reference state as previous path end point
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              //Use two points that make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);


          for (int i = 0; i < ptsx.size(); i++) {

              // Shift car reference angle to 0 debrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          // Create spline and set (x,y) points to the spline
          tk::spline s;
          s.set_points(ptsx, ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous points from the last time
          for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous points
          // Here we will always output 50 points
          for (int i = 1; i <= 50-previous_path_x.size(); i++) {
              double N = (target_dist/(0.02*ref_vel/2.24));
              double x_point = x_add_on+(target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;
              double x_ref = x_point;
              double y_ref = y_point;

              x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }
		/*************************
        *************************/
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
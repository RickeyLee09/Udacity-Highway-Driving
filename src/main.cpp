#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helpers.h"

using namespace std;

using json = nlohmann::json;

// global variable initialization
int LANE_WIDTH = 4;
float MAX_SPEED    = 49.5;
float MPS_TO_MPH = 2.23694;
float MPH_TO_MPS = 0.44704;
int OCCUPIED_COST = 100000;
int VALID_LANE_COST  =   1000;
int CHANGE_LANE_OFFSET = 500;

enum ACTION
{
    STAY_IN_LANE = 0,
    CHANGE_LANE_LEFT = 1,
    CHANGE_LANE_RIGHT = 2
};

bool changeLanePermission(double other_car_distance, double future_distance){
    if(other_car_distance > -15 && other_car_distance < 35){
        return true;
    }
    else if(future_distance > -15 && future_distance < 35){
        return true;
    }
    else{
        return false;
    }
}


enum ACTION perception(double my_car_s, double my_future_s, int lane, std::vector<std::vector<double> >& sensor_fusion,
                       int prev_size, double & speed) {

    enum ACTION action;

    double car_ahead_speed = 0;

    // initialize
    std::map<int, int> lane_cost;
    lane_cost[0] = 0;
    lane_cost[1] = 0;
    lane_cost[2] = 0;
    lane_cost[3] = OCCUPIED_COST;
    lane_cost[-1] = OCCUPIED_COST;

    // Keep track of the closest car in each lane.
    std::vector<double> closest_distance_in_lane(3, 500.0);

    for (unsigned int i = 0; i < sensor_fusion.size(); ++i) {
        double other_vehicle_data[4];

        for(int j=0; j<4; j++){
            other_vehicle_data[j] = sensor_fusion[i][j+3];
        }

        double other_car_mps = sqrt( other_vehicle_data[0] * other_vehicle_data[0] + other_vehicle_data[1] * other_vehicle_data[1]);
        double other_car_predicted_s = other_vehicle_data[2] + static_cast<double>(prev_size) * 0.02 * other_car_mps;

        // use d value to determine the lane
        int other_car_lane = int(floor(other_vehicle_data[3] / LANE_WIDTH));

        // distance in s to the other car.
        double other_car_distance = other_vehicle_data[2] - my_car_s;
        if ((other_car_distance < -25) || (other_car_distance > 75)) {
            continue;
        }

        // Keep track of the closest distance in each lane
        double future_distance = other_car_predicted_s - my_future_s;
        if ((future_distance > 0) && (future_distance < closest_distance_in_lane[other_car_lane])) {
            closest_distance_in_lane[other_car_lane] = future_distance;
        }

        // Penalize the lanes with cars that are too close.
        if(changeLanePermission(other_car_distance, future_distance)){
            lane_cost[other_car_lane] += VALID_LANE_COST;

            if (lane == other_car_lane) {
                car_ahead_speed = other_car_mps * MPS_TO_MPH;
            }
        }
     }

    // Whether to make a lane change
    if (lane_cost[lane] > CHANGE_LANE_OFFSET) {
        // If the lane change cost is to high, we slow down and remain at current lane
        if ((lane_cost[lane-1] >= VALID_LANE_COST) && (lane_cost[lane+1] >= VALID_LANE_COST)) {
            action = STAY_IN_LANE;
            speed = car_ahead_speed - 1.0;
        }
        else if ((lane_cost[lane-1] < VALID_LANE_COST) && (lane_cost[lane+1] < VALID_LANE_COST)) {
            // by default, change the lane left
            action = CHANGE_LANE_LEFT;
            speed  = MAX_SPEED;
        }
        // change lane left
        else if (lane_cost[lane-1] < VALID_LANE_COST) {
            action = CHANGE_LANE_LEFT;
            speed  = MAX_SPEED;
        }
        // change lane right
        else if (lane_cost[lane+1] < VALID_LANE_COST) {
            action = CHANGE_LANE_RIGHT;
            speed  = MAX_SPEED;
        }
    }
    // If lane change is not needed, we stay in current lane
    else {
        action = STAY_IN_LANE;
        speed  = MAX_SPEED;
    }

    return action;
}



int main() {
  uWS::Hub h;

  double current_speed = 0.0f;
  int current_lane  = 1;

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

  h.onMessage([&current_lane, &current_speed, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
   uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
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
         std::vector<std::vector<double> > sensor_fusion = j[1]["sensor_fusion"];

         int prev_size = previous_path_x.size();


         // Cost function.
         double recommended_speed;
         enum ACTION recommended_action = perception(car_s, end_path_s, current_lane, sensor_fusion, prev_size,
                                                              recommended_speed);

         // status machine
         switch (recommended_action) {
            case CHANGE_LANE_LEFT:
                current_lane -= 1;
                break;

            case CHANGE_LANE_RIGHT:
                current_lane += 1;
                break;

            case STAY_IN_LANE:
                break;
        }


        if (current_speed < recommended_speed) {
            current_speed = std::min(current_speed+0.3, double(MAX_SPEED));
        }
        else if (current_speed > recommended_speed) {
            current_speed = std::max(current_speed-0.3, 0.0);
        }

        json msgJson;

        vector<double> path_x;
        vector<double> path_y;

        double ref_x = car_x;
        double ref_y = car_y;
        double ref_yaw = deg2rad(car_yaw);

        // Spline library
        if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            path_x.push_back(prev_car_x);
            path_x.push_back(car_x);
            path_y.push_back(prev_car_y);
            path_y.push_back(car_y);
        }
        else {

            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            path_x.push_back(ref_x_prev);
            path_x.push_back(ref_x);
            path_y.push_back(ref_y_prev);
            path_y.push_back(ref_y);
        }

        // Generate more points
        for (unsigned int i = 0; i < 3; ++i) {
            int step = 60;
            int s_ahead = i * step + step;

            std::vector<double> next_wp = getXY(car_s + s_ahead,
                ((LANE_WIDTH / 2) + LANE_WIDTH * current_lane),
                map_waypoints_s,
                map_waypoints_x,
                map_waypoints_y);

            path_x.push_back(next_wp[0]);
            path_y.push_back(next_wp[1]);
        }

        // Coordinate transform to vehicle coordinate
        for (unsigned int i = 0; i < path_x.size(); ++i) {
            double shift_x = path_x[i] - ref_x;
            double shift_y = path_y[i] - ref_y;

            path_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            path_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
        }

        tk::spline s;
        s.set_points(path_x, path_y);

        std::vector<double> next_x_vals;
        std::vector<double> next_y_vals;

        for (unsigned int i = 0; i < previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
        }

        //  Euclidian distace Calculation
        double target_x = 100.0;
        double target_y = s(target_x);
        double target_dist = sqrt(target_x * target_x + target_y * target_y);

        double x_add_on = 0;

        for (unsigned int i = 1; i <= 75 - previous_path_x.size(); ++i) {
            double N = (target_dist / (0.02 * current_speed * MPH_TO_MPS));
            double x_point = x_add_on + (target_x / N);
            double y_point = s(x_point);

            x_add_on = x_point;

            // Coordinate transform to world coordinate
            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
        }

        // pass the points along to the sim.
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


  //
  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  //
h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
{
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


  //
  // handle socekt connection
  //
h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
{
    std::cout << "Connected!!!" << std::endl;
});


  //
  // handle socekt disconnection
  //
h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
{
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
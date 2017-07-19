#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include <utility>
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::map;
using std::pair;
using json = nlohmann::json;

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

string hasData(string s) 
{
    auto found_null = s.find("null");
    auto b1         = s.find_first_of("[");
    auto b2         = s.find_first_of("}");
    if (found_null != string::npos) 
    {
        return "";
    } 
    else if (b1 != string::npos && b2 != string::npos) 
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1) * (x2-x1) + (y2-y1) * (y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
    double closestLen   = 100000.0; 
    int closestWaypoint = 0;
    for (int i=0; i<maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist  = distance(x, y, map_x, map_y);
        if (dist < closestLen)
        {
            closestLen      = dist;
            closestWaypoint = i;
        }
    }
    return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
    double map_x        = maps_x[closestWaypoint];
    double map_y        = maps_y[closestWaypoint];
    double heading      = atan2((map_y-y), (map_x-x));
    double angle        = abs(theta-heading);
    if (angle > pi()/4)
    {
        closestWaypoint++;
    }
    return closestWaypoint;
}

vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
    int prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = maps_x.size() - 1;
    }
    double n_x         = maps_x[next_wp] - maps_x[prev_wp];
    double n_y         = maps_y[next_wp] - maps_y[prev_wp];
    double x_x         = x - maps_x[prev_wp];
    double x_y         = y - maps_y[prev_wp];
    double proj_norm   = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x      = proj_norm * n_x;
    double proj_y      = proj_norm * n_y;
    double frenet_d    = distance(x_x, x_y, proj_x, proj_y);
    double center_x    = 1000 - maps_x[prev_wp];
    double center_y    = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);
    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }
    double frenet_s = 0;
    for (int i=0; i<prev_wp; i++)
    {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
    }
    frenet_s += distance(0, 0, proj_x, proj_y);
    return {frenet_s, frenet_d};
}

vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
    int prev_wp = -1;
    while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1)))
    {
        prev_wp++;
    }
    int wp2             = (prev_wp+1) % maps_x.size();
    double heading      = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    double seg_s        = s - maps_s[prev_wp];
    double seg_x        = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y        = maps_y[prev_wp] + seg_s * sin(heading);
    double perp_heading = heading - pi() / 2;
    double x            = seg_x + d * cos(perp_heading);
    double y            = seg_y + d * sin(perp_heading);
    return {x, y};
}

/*
 ***************************************************************************************************************
 */

// Calculates the coefficients of a jerk-minimizing transition and then calculates the points along the path
vector<double> minimum_jerk_path(vector<double> start, vector<double> end, double max_time)
{
    // Load matrix, vector and compute coefficients
    MatrixXd A = MatrixXd(3,3);
    VectorXd b = VectorXd(3);
    VectorXd x = VectorXd(3);

    double t  = max_time;
    double t2 = t * t;
    double t3 = t * t2;
    double t4 = t * t3;
    double t5 = t * t4;

    A <<   t3,    t4,    t5,
         3*t2,  4*t3,  5*t4,
         6*t,  12*t2, 20*t3;

    b << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
         end[1] - (start[1] + start[2] * t),
         end[2] - start[2];

    x = A.inverse() * b;

    double a0 = start[0];
    double a1 = start[1];
    double a2 = start[2] / 2.0;
    double a3 = x[0];
    double a4 = x[1];
    double a5 = x[2];

    // Iterate on time and produce path result
    vector<double> result;

    for (double t=0.0; t<max_time; t+=0.02)
    {
        double t2 = t * t;
        double t3 = t * t2;
        double t4 = t * t3;
        double t5 = t * t4;
    
        double r = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
        result.push_back(r);
    }

    return result;
}


// Converts an enumerated lane to an absolute measure in terms of Frenet d coordinate (lanes being counting numbers)
double convertLaneToD(int lane)
{
    return 2.0 + 4.0 * (double)(lane - 1);
}

// Cost of a change of lane to the left
double costOfLaneChangeLeft()
{

}

// Cost of a change of lane to the right
double costOfLaneChangeRight()
{

}

// Cost of maintaining straight course
double costOfStraightCourse()
{

}

// Cost of increasing speed to speed limit
double costOfAcceleratingToSpeedLimit()
{

}



/*
 ***************************************************************************************************************
 */

int main() {
    uWS::Hub h;

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    string map_file_ = "../data/highway_map.csv";
    double max_s     = 6945.554;
    ifstream in_map_(map_file_.c_str(), ifstream::in);
    string line;

    while (getline(in_map_, line)) 
    {
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

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
    {
        if (length && length > 2 && data[0] == '4' && data[1] == '2') 
        {
            auto s = hasData(data);
            if (s != "") 
            {
                auto j       = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") 
                {
                    double car_x         = j[1]["x"];
                    double car_y         = j[1]["y"];
                    double car_s         = j[1]["s"];
                    double car_d         = j[1]["d"];
                    double car_yaw       = j[1]["yaw"];
                    double car_speed     = j[1]["speed"];
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    double end_path_s    = j[1]["end_path_s"];
                    double end_path_d    = j[1]["end_path_d"];
                    auto sensor_fusion   = j[1]["sensor_fusion"];

                    /* TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                     ****************************************************************************************************
                     */
                  
                    // Make the whole telemetry package available to the methods above for cost...
                    auto telemetry_data = j[1];


                    // Look at sensor fusion data -- THIS IS JUST MESSING AROUND
                    for (int i=0; i<sensor_fusion.size(); i++)
                    {
                        int id    = sensor_fusion[i][0];
                        double x  = sensor_fusion[i][1];
                        double y  = sensor_fusion[i][2];
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double s  = sensor_fusion[i][5];
                        double d  = sensor_fusion[i][6];
                    }
                  
                    // Choose an action based on the costs of various options
                    map<string, double> path_options = { {"left", 0.0},
                                                         {"right", 0.0},
                                                         {"keep", 0.0},
                                                         {"limit", 0.0} };
                    path_options["left"]  = costOfLaneChangeLeft();
                    path_options["right"] = costOfLaneChangeRight();
                    path_options["keep"]  = costOfStraightCourse();
                    path_options["limit"] = costOfAcceleratingToSpeedLimit();

                    // Find lowest cost
                    


                    // Choose a start point (where we are now) and an end-point (may be based on costs over ensemble)


                    // Choose initial and final conditions for the minimum jerk interpolator (always using zero acceleration endpoints)
                    double start_pos_s = car_s;
                    double start_vel_s = 0.0;
                    double end_pos_s   = car_s + 30.0;
                    double end_vel_s   = 0.0;

                    double d_pos = convertLaneToD(1);
                    double start_pos_d = d_pos;
                    double start_vel_d = 0.0;
                    double end_pos_d   = d_pos;
                    double end_vel_d   = 0.0;

                    // Generate path data (Frenet)
                    vector<double> next_s_vals = minimum_jerk_path({start_pos_s, start_vel_s, 0.0}, 
                                                                   {end_pos_s,   end_vel_s,   0.0}, 
                                                                   1.0);
                    vector<double> next_d_vals = minimum_jerk_path({start_pos_d, start_vel_d, 0.0}, 
                                                                   {end_pos_d,   end_vel_d,   0.0}, 
                                                                   1.0);

                    // Convert back to map coordinates
                    vector<double> next_x_pre_vals = {};
                    vector<double> next_y_pre_vals = {};
                    for (int i=0; i<next_s_vals.size(); i++)
                    {
                        vector<double> xy = getXY(next_s_vals[i],
                                                  next_d_vals[i],
                                                  map_waypoints_s,
                                                  map_waypoints_x,
                                                  map_waypoints_y);
                        next_x_pre_vals.push_back(xy[0]);
                        next_y_pre_vals.push_back(xy[1]);
                    }

                    // Use spline interpolator to minimize acceleration at derivative discontinuities
                    vector<double> time_pre_vals = {};
                    for (int i=0; i<next_s_vals.size(); i++)
                    {
                        time_pre_vals.push_back((double)i / 50.0);
                    }

                    tk::spline sx;
                    sx.set_points(time_pre_vals, next_x_pre_vals);

                    tk::spline sy;
                    sy.set_points(time_pre_vals, next_y_pre_vals);
                    
                    vector<double> next_x_vals = {};
                    vector<double> next_y_vals = {};

                    for (int i=0; i<next_s_vals.size(); i++)
                    {
                        double t = (double)i / 50.0;
                        next_x_vals.push_back(sx(t));
                        next_y_vals.push_back(sy(t));
                    }

                    // Send to the simulator
                    json msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    /*
                     ****************************************************************************************************
                     */

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } 
            else 
            {
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) 
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) 
        {
            res->end(s.data(), s.length());
        } 
        else 
        {
            res->end(nullptr, 0);
        }
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


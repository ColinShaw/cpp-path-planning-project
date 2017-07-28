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


#define MAP_FILE                "../data/highway_map.csv"

#define NUM_RESAMPLED_WAYPOINTS 10000

#define PATH_PLAN_SECONDS       2.5
#define PATH_PLAN_INCREMENT     0.02

#define LANE_CHANGE_COST_CONST  0.6

#define MAX_SPEED_M_S           19.5

#define SPEED_INCREMENT         0.0
#define LANE_CHANGE_CONSTANT    1.0 //0.95

#define DISTANCE_ADJUSTMENT     2.5
#define DISTANCE_THRESHOLD      20.0


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::map;
using json = nlohmann::json;


// Type for saving state between evaluation cycles with the simulator
struct save_state_t
{
    double last_s;
    double last_d;
    double last_speed;
};

// Type for the data about the other cars
struct other_car_t
{
    int    id;
    int    car_l;
    double car_s;
    double car_speed;
};

// Telemetry type encapsulating the data we need for determining course
struct telemetry_t 
{
    int    car_l;
    double car_s;
    double car_speed;
    vector<other_car_t> other_cars; 
};

// Setpoint type for the controls we are returning
struct setpoint_t
{
    double start_pos_s;
    double start_vel_s;
    double end_pos_s;
    double end_vel_s;
    int    start_pos_l;
    int    end_pos_l;
};

// Type encapsulating x,y pairs and last s,d from jerk minimization
struct jerk_return_t 
{
    vector<double> path_x;
    vector<double> path_y;
    double last_s;
    double last_d;
};


constexpr double pi()    { return M_PI; }
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

// Calculates jerk minimizing path
vector<double> computeMinimumJerk(vector<double> start, vector<double> end, double max_time, double time_inc)
{
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

    vector<double> result;
    for (double t=time_inc; t<max_time+0.001; t+=time_inc) 
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

// Explicit conversion of lane number to Frenet d-coordinate
double convertLaneToD(int lane)
{
    if (lane == 1)
    {
        return 2.0;
    }
    if (lane == 2)
    {
        return 6.0;
    }
    if (lane == 3)
    {
        return 10.0;
    }
    return 0;
}

// Fuzzy conversion of Frenet d-coordinate to the nearest enumerated lane
int convertDToLane(double d)
{
    if (d<4)
    {
         return 1;
    }
    if (d>=4.0 && d<8.0)
    {
        return 2;
    }
    if (d>=8.0)
    {
        return 3;
    }
    return 0;
}

// Determine distance to closest car in front of us in a given lane
double distanceToClosestCarInFront(telemetry_t telemetry_data, int lane)
{
    double closest = 1000000.0;
    for (int i=0; i<telemetry_data.other_cars.size(); i++)
    {
        if (telemetry_data.other_cars[i].car_l == lane)
        {
            double diff = telemetry_data.other_cars[i].car_s - telemetry_data.car_s;
            if (diff > 0.0 && diff < closest)
            {
                closest = diff;
            }
        }
    }
    return closest;
}

// Determine distance to the closest car behind us in a given lane
double distanceToClosestCarBehind(telemetry_t telemetry_data, int lane)
{
    double closest = 1000000.0;
    for (int i=0; i<telemetry_data.other_cars.size(); i++)
    {
        if (telemetry_data.other_cars[i].car_l == lane)
        {
            double diff = telemetry_data.car_s - telemetry_data.other_cars[i].car_s;
            if (diff > 0.0 && diff < closest)
            {
                closest = diff;
            }
        }
    }
    return closest;
}

// Cost of a change of lane to the left
double costOfLaneChangeLeft(telemetry_t telemetry_data)
{
    if (telemetry_data.car_l == 1)
    {
        return 1000.0; 
    }
    double front_dist  = distanceToClosestCarInFront(telemetry_data, telemetry_data.car_l-1);
    double behind_dist = distanceToClosestCarBehind(telemetry_data, telemetry_data.car_l-1);
    if (front_dist != 0.0 && behind_dist != 0.0)
    {
        return LANE_CHANGE_COST_CONST * (1.0 / front_dist + 1.0 / behind_dist);
    }
    return 1000.0;
}

// Cost of a change of lane to the right
double costOfLaneChangeRight(telemetry_t telemetry_data)
{
    if (telemetry_data.car_l == 3)
    {
        return 1000.0;
    }
    double front_dist  = distanceToClosestCarInFront(telemetry_data, telemetry_data.car_l+1);
    double behind_dist = distanceToClosestCarBehind(telemetry_data, telemetry_data.car_l+1);
    if (front_dist != 0.0 && behind_dist != 0.0)
    {
        return LANE_CHANGE_COST_CONST * (1.0 / front_dist + 1.0 / behind_dist);
    }
    return 1000.0;
}

// Cost of maintaining straight course
double costOfStraightCourse(telemetry_t telemetry_data)
{
    double front_dist = distanceToClosestCarInFront(telemetry_data, telemetry_data.car_l);
    if (front_dist != 0.0)
    {
        return LANE_CHANGE_CONSTANT * 1.0 / front_dist;
    }
    return 1000.0;
}

// Determine new setpoints whilst going on the left course 
setpoint_t determineNewLeftCourseSetpoints(telemetry_t telemetry_data)
{
    // Lane shift to the left
    setpoint_t retval = {
        telemetry_data.car_s,
        telemetry_data.car_speed,
        telemetry_data.car_s + LANE_CHANGE_CONSTANT * PATH_PLAN_SECONDS * telemetry_data.car_speed,
        telemetry_data.car_speed - SPEED_INCREMENT,
        telemetry_data.car_l,
        telemetry_data.car_l - 1 
    };
    return retval;
}

// Determine new setpoints whilst going on the right course 
setpoint_t determineNewRightCourseSetpoints(telemetry_t telemetry_data)
{
    // Lane shift to the right
    setpoint_t retval = {
        telemetry_data.car_s,
        telemetry_data.car_speed,
        telemetry_data.car_s + LANE_CHANGE_CONSTANT * PATH_PLAN_SECONDS * telemetry_data.car_speed,
        telemetry_data.car_speed - SPEED_INCREMENT,
        telemetry_data.car_l,
        telemetry_data.car_l + 1 
    };
    return retval;
}

// Determine new setpoints whilst going on the straight course 
setpoint_t determineNewStraightCourseSetpoints(telemetry_t telemetry_data)
{
    double car_in_front_dist  = distanceToClosestCarInFront(telemetry_data, telemetry_data.car_l);
    double car_in_front_adj   = DISTANCE_ADJUSTMENT * (car_in_front_dist - DISTANCE_THRESHOLD);
    if (car_in_front_adj > 5.0)
    {
        car_in_front_adj = 5.0;
    }
    if (car_in_front_adj < -5.0)
    {
        car_in_front_adj = -5.0;
    }
    double speed_start        = telemetry_data.car_speed;
    if (speed_start > MAX_SPEED_M_S)
    {
        speed_start = MAX_SPEED_M_S;
    }
    double speed_end = speed_start + car_in_front_adj;
    if (speed_end > MAX_SPEED_M_S)
    {
        speed_end = MAX_SPEED_M_S;
    }

cout << "speed_start " << speed_start << endl;
cout << "speed_end " << speed_end << endl << endl;

    setpoint_t retval = {
        telemetry_data.car_s,
        speed_start,
        telemetry_data.car_s + PATH_PLAN_SECONDS * 0.5 * (speed_start + speed_end), 
        speed_end,
        telemetry_data.car_l,
        telemetry_data.car_l 
    };
    return retval;
}

// Calculate the lowest cost action
string calculateLowestCostAction(telemetry_t telemetry_data)
{
    double left_cost  = costOfLaneChangeLeft(telemetry_data);
    double keep_cost  = costOfStraightCourse(telemetry_data);
    double right_cost = costOfLaneChangeRight(telemetry_data);

cout << "costs: " << left_cost << " - " << keep_cost << " - " << right_cost << endl;

    map<double, string> cost_map = { {left_cost,  "left"},
                                     {keep_cost,  "keep"},
                                     {right_cost, "right"} };

    // First value is the lowest cost since it is a priority queue on key
    map<double, string>::iterator cost_map_iterator;
    cost_map_iterator = cost_map.begin();
    string action = cost_map_iterator->second;
    return action;
}

// Compute minimum jerk path and convert to map coordinates
jerk_return_t computeMinimumJerkMapPath(setpoint_t new_setpoints,
                                                 vector<double> map_waypoints_s,
                                                 vector<double> map_waypoints_x,
                                                 vector<double> map_waypoints_y)
{
    // Conditions for minimum jerk in s (zero start/end acceleration) 
    double start_pos_s = new_setpoints.start_pos_s; 
    double start_vel_s = new_setpoints.start_vel_s; 
    double end_pos_s   = new_setpoints.end_pos_s; 
    double end_vel_s   = new_setpoints.end_vel_s; 

    // Conditions for minimum jerk in d (zero start/end acceleration and velocity, indexing by lane) 
    double start_pos_d = convertLaneToD(new_setpoints.start_pos_l);
    double end_pos_d   = convertLaneToD(new_setpoints.end_pos_l);

    // Generate minimum jerk path in Frenet coordinates
    vector<double> next_s_vals = computeMinimumJerk({start_pos_s, start_vel_s, 0.0}, 
                                                    {end_pos_s,   end_vel_s,   0.0}, 
                                                    PATH_PLAN_SECONDS,
                                                    PATH_PLAN_INCREMENT);
    vector<double> next_d_vals = computeMinimumJerk({start_pos_d, 0.0, 0.0}, 
                                                    {end_pos_d,   0.0, 0.0}, 
                                                    PATH_PLAN_SECONDS,
                                                    PATH_PLAN_INCREMENT);

    // Convert Frenet coordinates to map coordinates
    vector<double> next_x_vals = {};
    vector<double> next_y_vals = {};
    for (int i=0; i<next_s_vals.size(); i++)
    {
        vector<double> xy = getXY(next_s_vals[i],
                                  next_d_vals[i],
                                  map_waypoints_s,
                                  map_waypoints_x,
                                  map_waypoints_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }
    jerk_return_t retval;
    retval.path_x = next_x_vals;
    retval.path_y = next_y_vals;
    retval.last_s = next_s_vals[next_s_vals.size()-1];
    retval.last_d = next_d_vals[next_d_vals.size()-1];
    return retval;
}

int main() {
    uWS::Hub h;

    // Load waypoints
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    string line;
    ifstream in_map_(MAP_FILE, ifstream::in);
    while (getline(in_map_, line)) 
    {
        istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
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

    // Spline interpolate the map waypoints
    map_waypoints_x.push_back(map_waypoints_x[0]);
    map_waypoints_y.push_back(map_waypoints_y[0]);
    map_waypoints_s.push_back(map_waypoints_s[0]);
    map_waypoints_dx.push_back(map_waypoints_dx[0]);
    map_waypoints_dy.push_back(map_waypoints_dy[0]);
    map_waypoints_x.push_back(map_waypoints_x[1]);
    map_waypoints_y.push_back(map_waypoints_y[1]);
    map_waypoints_s.push_back(map_waypoints_s[1]);
    map_waypoints_dx.push_back(map_waypoints_dx[1]);
    map_waypoints_dy.push_back(map_waypoints_dy[1]);

    vector<double> waypoint_spline_t = {};
    int map_waypoints_size = map_waypoints_x.size();
    for (int i=0; i<map_waypoints_size; i++)
    {
        double t = (double)i / (double)map_waypoints_size; 
        waypoint_spline_t.push_back(t);
    }

    tk::spline waypoint_spline_x;
    waypoint_spline_x.set_points(waypoint_spline_t, map_waypoints_x);
    tk::spline waypoint_spline_y;
    waypoint_spline_y.set_points(waypoint_spline_t, map_waypoints_y);
    tk::spline waypoint_spline_s;
    waypoint_spline_s.set_points(waypoint_spline_t, map_waypoints_s);
    tk::spline waypoint_spline_dx;
    waypoint_spline_dx.set_points(waypoint_spline_t, map_waypoints_dx);
    tk::spline waypoint_spline_dy;
    waypoint_spline_dy.set_points(waypoint_spline_t, map_waypoints_dy);
    
    vector<double> map_waypoints_x_new;
    vector<double> map_waypoints_y_new;
    vector<double> map_waypoints_s_new;
    vector<double> map_waypoints_dx_new;
    vector<double> map_waypoints_dy_new;

    for (int i=0; i<NUM_RESAMPLED_WAYPOINTS; i++)
    {
        double t = (double)i / (double)NUM_RESAMPLED_WAYPOINTS;
        map_waypoints_x_new.push_back(waypoint_spline_x(t));
        map_waypoints_y_new.push_back(waypoint_spline_y(t));
        map_waypoints_s_new.push_back(waypoint_spline_s(t));
        map_waypoints_dx_new.push_back(waypoint_spline_dx(t));
        map_waypoints_dy_new.push_back(waypoint_spline_dy(t));
    }

    map_waypoints_x  = map_waypoints_x_new;
    map_waypoints_y  = map_waypoints_y_new;
    map_waypoints_s  = map_waypoints_s_new;
    map_waypoints_dx = map_waypoints_dx_new;
    map_waypoints_dy = map_waypoints_dy_new;

    save_state_t save_state;

    // Respond to simulator telemetry messages
    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, 
                 &map_waypoints_dx, &map_waypoints_dy, &save_state]
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

                    // Some variables used here...
                    json msgJson;
                    setpoint_t new_setpoints;

                    // First path
                    if (previous_path_x.size() == 0)
                    {
                        vector<other_car_t> other_cars = {};
                        for (int i=0; i<sensor_fusion.size(); i++)
                        {
                            int id    = sensor_fusion[i][0];
                            double s  = sensor_fusion[i][5];
                            double d  = sensor_fusion[i][6];
                            int l     = convertDToLane(d);
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double speed = sqrt(vx*vx + vy*vy);
                            other_car_t oc = {id, l, s, speed};
                            other_cars.push_back(oc); 
                        }
                        int pos_l = convertDToLane(car_d);
                        telemetry_t telemetry_data = {pos_l, car_s, car_speed, other_cars};
                        new_setpoints = determineNewStraightCourseSetpoints(telemetry_data);
                        jerk_return_t jerk = computeMinimumJerkMapPath(new_setpoints,
                                                                       map_waypoints_s,
                                                                       map_waypoints_x,
                                                                       map_waypoints_y);
                        save_state.last_s = jerk.last_s;
                        save_state.last_d = jerk.last_d;
                        save_state.last_speed = new_setpoints.end_vel_s;
                        msgJson["next_x"] = jerk.path_x; 
                        msgJson["next_y"] = jerk.path_y;
                    }

                    // Nearing the end of driven path
                    else if (previous_path_x.size() < 15)
                    {
                        vector<other_car_t> other_cars = {};
                        for (int i=0; i<sensor_fusion.size(); i++)
                        {
                            int id    = sensor_fusion[i][0];
                            double s  = sensor_fusion[i][5];
                            double d  = sensor_fusion[i][6];
                            int l     = convertDToLane(d);
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double speed = sqrt(vx*vx + vy*vy);
                            other_car_t oc = {id, l, s, speed};
                            other_cars.push_back(oc); 
                        }					
                        double car_s = save_state.last_s;
                        double car_d = save_state.last_d;
                        double car_speed = save_state.last_speed;	
                        int pos_l = convertDToLane(car_d);
                        telemetry_t telemetry_data = {pos_l, car_s, car_speed, other_cars};
                        string action = calculateLowestCostAction(telemetry_data);
                        if (action == "left")
                        {
                            new_setpoints = determineNewLeftCourseSetpoints(telemetry_data);
                        }
                        else if (action == "keep")
                        {
                            new_setpoints = determineNewStraightCourseSetpoints(telemetry_data);
                        }
                        else if (action == "right")
                        {
                            new_setpoints = determineNewRightCourseSetpoints(telemetry_data);
                        }
                        vector<double> path_x = previous_path_x;
                        vector<double> path_y = previous_path_y;
                        jerk_return_t jerk = computeMinimumJerkMapPath(new_setpoints,
                                                                       map_waypoints_s,
                                                                       map_waypoints_x,
                                                                       map_waypoints_y);
                        save_state.last_s = jerk.last_s;
                        save_state.last_d = jerk.last_d;
                        save_state.last_speed = new_setpoints.end_vel_s;
                        for (int i=0; i<jerk.path_x.size(); i++)
                        {
                            path_x.push_back(jerk.path_x[i]);
                            path_y.push_back(jerk.path_y[i]);
                        }
                        msgJson["next_x"] = path_x; 
                        msgJson["next_y"] = path_y;
                    }

                    // Just resend the rest of the path back to sim
                    else
                    {
                        msgJson["next_x"] = previous_path_x; 
                        msgJson["next_y"] = previous_path_y;
                    }

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


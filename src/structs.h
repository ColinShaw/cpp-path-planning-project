#include <vector>

using std::vector;

// Type for the data about the other cars
struct other_car_t
{
    int    id;
    int    car_l;
    double car_s;
    double car_speed;
};

// We need a telemetry type encapsulating the data we need for determining course
struct telemetry_t 
{
    int    car_l;
    double car_s;
    double car_speed;
    vector<other_car_t> other_cars; 
};

// And a setpoint type for the controls we are returning
struct setpoint_t
{
    double start_pos_s;
    double start_vel_s;
    double end_pos_s;
    double end_vel_s;
    int    start_pos_l;
    int    end_pos_l;
};



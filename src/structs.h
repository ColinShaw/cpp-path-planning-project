#include <vector>

using std::vector;

// Type for saving state between evaluation cycles with the simulator
struct save_state_t
{
    double last_s;
    double last_d;
    int    last_front_car_id;
    double last_front_car_s;
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

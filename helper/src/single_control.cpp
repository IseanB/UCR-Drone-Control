#include "../single_control.h"

SingleDroneControl::SingleDroneControl(float max_velocity, float max_acceleration, int droneID) :
    default_max_velocity(max_velocity),
    default_max_acceleration(max_acceleration), 
    current_trajectory(nullptr),
    curr_position(nullptr), 
    curr_state(IDLE){
        if(droneID == -1){}
}
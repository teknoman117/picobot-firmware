#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <cstdint>

void setup_motors();
void setup_encoders();

void set_motor_left_speed(float speed);
void set_motor_right_speed(float speed);
void set_motor_lidar_speed(float speed);
void set_motor_tilt_position(float position);

int32_t get_motor_velocity();

#endif
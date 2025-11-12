#include "motors.hpp"

#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/pwm.h>

//#include "quadrature_encoder.pio.h"

#define MOTOR_LEFT_PWM 22
#define MOTOR_LEFT_DIR 21
#define MOTOR_LEFT_ENCODER 20

#define MOTOR_RIGHT_PWM 19
#define MOTOR_RIGHT_DIR 18
#define MOTOR_RIGHT_ENCODER 16

#define MOTOR_LIDAR_PWM 15

#define MOTOR_TILT_PWM 0

#define MOTOR_PWM_WRAP 5000
#define MOTOR_DEADZONE 50

namespace {
    int32_t previous_left_encoder = 0;
    int32_t previous_right_encoder = 0;
}

void setup_motors() {
    // set up the motor GPIOs and PWM
    gpio_init(MOTOR_LEFT_DIR);
    gpio_set_dir(MOTOR_LEFT_DIR, GPIO_OUT);
    gpio_put(MOTOR_LEFT_DIR, false);

    gpio_init(MOTOR_RIGHT_DIR);
    gpio_set_dir(MOTOR_RIGHT_DIR, GPIO_OUT);
    gpio_put(MOTOR_RIGHT_DIR, false);

    gpio_set_function(MOTOR_LEFT_PWM, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_LEFT_PWM), MOTOR_PWM_WRAP);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_LEFT_PWM), true);

    gpio_set_function(MOTOR_RIGHT_PWM, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM), MOTOR_PWM_WRAP);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM), true);

    gpio_set_function(MOTOR_LIDAR_PWM, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_LIDAR_PWM), MOTOR_PWM_WRAP);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_LIDAR_PWM), true);

    auto config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, 125);
    pwm_config_set_wrap(&config, 20000);

    gpio_set_function(MOTOR_TILT_PWM, GPIO_FUNC_PWM);
    pwm_init(pwm_gpio_to_slice_num(MOTOR_TILT_PWM), &config, true);
}

void setup_encoders() {
    //pio_add_program(pio0, &quadrature_encoder_program);

    //quadrature_encoder_program_init(pio0, 0, MOTOR_LEFT_ENCODER_A, 0);
    //quadrature_encoder_program_init(pio0, 1, MOTOR_RIGHT_ENCODER_A, 0);
}

void set_motor_left_speed(float speed_) {
    auto speed = (speed_ > 1.f ? 1.f : speed_ < -1.f ? -1.f : speed_)
            * (float) (MOTOR_PWM_WRAP - MOTOR_DEADZONE);

    if (speed > 0) {
        gpio_put(MOTOR_LEFT_DIR, false);
        pwm_set_gpio_level(MOTOR_LEFT_PWM, speed + MOTOR_DEADZONE);
    } else if (speed < 0) {
        gpio_put(MOTOR_LEFT_DIR, true);
        pwm_set_gpio_level(MOTOR_LEFT_PWM, -speed - MOTOR_DEADZONE);
    } else {
        gpio_put(MOTOR_LEFT_DIR, false);
        pwm_set_gpio_level(MOTOR_LEFT_PWM, 0);
    }
}

void set_motor_right_speed(float speed_) {
    auto speed = (speed_ > 1.f ? 1.f : speed_ < -1.f ? -1.f : speed_)
            * (float) (MOTOR_PWM_WRAP - MOTOR_DEADZONE);

    if (speed > 0) {
        gpio_put(MOTOR_RIGHT_DIR, true);
        pwm_set_gpio_level(MOTOR_RIGHT_PWM, speed + MOTOR_DEADZONE);
    } else if (speed < 0) {
        gpio_put(MOTOR_RIGHT_DIR, false);
        pwm_set_gpio_level(MOTOR_RIGHT_PWM, -speed - MOTOR_DEADZONE);
    } else {
        gpio_put(MOTOR_RIGHT_DIR, false);
        pwm_set_gpio_level(MOTOR_RIGHT_PWM, 0);
    }
}

void set_motor_lidar_speed(float speed_) {
    auto speed = (speed_ > 1.f ? 1.f : speed_ < -1.f ? -1.f : speed_)
            * (float) (MOTOR_PWM_WRAP - MOTOR_DEADZONE);

    if (speed > 0) {
        pwm_set_gpio_level(MOTOR_LIDAR_PWM, speed + MOTOR_DEADZONE);
    } else if (speed < 0) {
        pwm_set_gpio_level(MOTOR_LIDAR_PWM, -speed - MOTOR_DEADZONE);
    } else {
        pwm_set_gpio_level(MOTOR_LIDAR_PWM, 0);
    }
}

void set_motor_tilt_position(float position_) {
    auto position = (position_ > 1.f ? 1.f : position_ < -1.f ? -1.f : position_) * 750.f;
    pwm_set_gpio_level(MOTOR_TILT_PWM, ((int16_t) position) + 1500);
}

int32_t get_motor_velocity() {
    /*int32_t left_encoder = quadrature_encoder_get_count_nonblocking(pio0, 0, previous_left_encoder);
    int32_t right_encoder = quadrature_encoder_get_count_nonblocking(pio0, 1, previous_right_encoder);
    int32_t left_encoder_delta = left_encoder - previous_left_encoder;
    int32_t right_encoder_delta = right_encoder - previous_right_encoder;
    previous_left_encoder = left_encoder;
    previous_right_encoder = right_encoder;
    return (left_encoder_delta + right_encoder_delta) / 2;*/
    return 0;
}
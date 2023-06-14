#pragma once

namespace ProjectConfig
{
    short const left_step_pin = 27;
    short const left_dir_pin = 14;
    short const left_en_pin = 33;
    short const right_step_pin = 25;
    short const right_dir_pin = 26;
    short const right_en_pin = 32;
    
    int const full_steps_per_rev = 200;
    int const microstep_divs = 16;
    float const wheel_circumference_mm = 220;
    float const wheel_distance_mm = 150;
    
    float const motor_max_rpm = 60;
    float const motor_accel = 500;
}

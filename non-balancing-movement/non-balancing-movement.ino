#include <ESP_FlexyStepper.h>

const unsigned left_step_pin = 27;
const unsigned left_dir_pin = 14;
const unsigned right_step_pin = 25;
const unsigned right_dir_pin = 26;

const unsigned full_steps_per_rev = 200;
const unsigned microstep_divs = 16;
const unsigned wheel_circumference_mm = 220;

const unsigned motor_max_speed_mm_per_s = 500;
const unsigned motor_accel = 500;

ESP_FlexyStepper left_motor, right_motor;

void setup() {
    Serial.begin(115200);
    delay(500); // Allow time for Arduino serial monitor to connect

    left_motor.connectToPins(left_step_pin, left_dir_pin);
    left_motor.setStepsPerMillimeter(full_steps_per_rev * microstep_divs / wheel_circumference_mm);
    left_motor.setSpeedInMillimetersPerSecond(motor_max_speed_mm_per_s);
    left_motor.setAccelerationInMillimetersPerSecondPerSecond(motor_accel);
    left_motor.setDecelerationInMillimetersPerSecondPerSecond(motor_accel);
    left_motor.startAsService(0);

    right_motor.connectToPins(right_step_pin, right_dir_pin);
    right_motor.setStepsPerMillimeter(full_steps_per_rev * microstep_divs / wheel_circumference_mm);
    right_motor.setSpeedInMillimetersPerSecond(motor_max_speed_mm_per_s);
    right_motor.setAccelerationInMillimetersPerSecondPerSecond(motor_accel);
    right_motor.setDecelerationInMillimetersPerSecondPerSecond(motor_accel);
    right_motor.startAsService(0);
}

void loop() {
    static bool forward = true;

    // Serial.printf("Current position (mm): %f, %f\n", left_motor.getCurrentPositionInMillimeters(), right_motor.getCurrentPositionInMillimeters());
    
    if (left_motor.getDistanceToTargetSigned() == 0) {
        delay(1000);
        left_motor.setTargetPositionInMillimeters(forward ? 100 : -100);
        right_motor.setTargetPositionInMillimeters(forward ? 100 : -100);
        forward = !forward;
    }

    left_motor.processMovement();
    right_motor.processMovement();
}

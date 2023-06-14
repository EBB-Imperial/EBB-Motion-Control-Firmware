#include <BasicStepperDriver.h>
#include <MultiDriver.h>
#include "project_config.h"
#include "commands.h"
::
BasicStepperDriver left_motor(ProjectConfig::full_steps_per_rev, ProjectConfig::left_dir_pin, ProjectConfig::left_step_pin);
BasicStepperDriver right_motor(ProjectConfig::full_steps_per_rev, ProjectConfig::right_dir_pin, ProjectConfig::right_step_pin);
MultiDriver motors(left_motor, right_motor);

static float const turn_circumference_mm = ProjectConfig::wheel_distance_mm * 3.14;
static float const microsteps_per_mm = ProjectConfig::full_steps_per_rev * ProjectConfig::microstep_divs / ProjectConfig::wheel_circumference_mm;

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(100);
    
    Serial2.begin(1000000);
    Serial2.setTimeout(100);
    
    delay(500); // Allow time for Arduino serial monitor to connect

    left_motor.begin(ProjectConfig::motor_max_rpm, ProjectConfig::microstep_divs);
    right_motor.begin(ProjectConfig::motor_max_rpm, ProjectConfig::microstep_divs);
    left_motor.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, ProjectConfig::motor_accel, ProjectConfig::motor_accel);
    right_motor.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, ProjectConfig::motor_accel, ProjectConfig::motor_accel);
}

void loop()
{
    MovementCommand action;
    
    if (Serial.available()) {
        int c = Serial.read();
        Serial.printf("Received 0x%04x\n", c);

        switch (c) {
            case 0x00: action = MovementCommand::CMD_HCF; break;
            case 0x01: action = MovementCommand::CMD_STATUS; break;
            case 0x02: action = MovementCommand::CMD_STRAIGHT; break;
            case 0x03: action = MovementCommand::CMD_TURN; break;
            default: action = MovementCommand::CMD_UNKNOWN; break;
        }

        if (action == MovementCommand::CMD_HCF) {
            motors.stop();
        } else if (action == MovementCommand::CMD_STATUS) {
            Serial.write(motors.isRunning());
        } else if (action == MovementCommand::CMD_STRAIGHT) {
            byte dist_param[2];
            if (Serial.readBytes(dist_param, 2) == 2) {
                int16_t dist_mm = dist_param[0] | dist_param[1] << 8;
                long steps = dist_mm * microsteps_per_mm;
                motors.startMove(steps, steps);
            }
        } else if (action == MovementCommand::CMD_TURN) {
            byte angle_param[2];
            if (Serial.readBytes(angle_param, 2) == 2) {
                int16_t angle_deg = angle_param[0] | angle_param[1] << 8;
                float dist_mm = angle_deg / 360.0 * turn_circumference_mm;
                long steps = dist_mm * microsteps_per_mm;
                motors.startMove(steps, -steps);
            }
        } else if (action == MovementCommand::CMD_UNKNOWN) {
            ; // Ignore invalid commands
        }
    }
    
    motors.nextAction();
}

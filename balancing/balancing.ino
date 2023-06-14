#include <Wire.h>
#include <MPU6050_light.h>
#include <AutoPID.h>
#include <ContinuousStepper.h>

static MPU6050 imu(Wire);

static double pitch, pitch_setpoint, pitch_action;
static AutoPID pitch_controller(&pitch, &pitch_setpoint, &pitch_action, -10000, 10000, 7, 1, 2);

static ContinuousStepper left_stepper, right_stepper;

hw_timer_t* stepper_timer = nullptr;

void ARDUINO_ISR_ATTR on_stepper_timer()
{
    left_stepper.loop();
    right_stepper.loop();
}

void setup()
{
    Serial.begin(2000000);
    delay(500);
    
    Wire.begin(33, 32, 400000);

    imu.begin();
    imu.calcOffsets();
    imu.setFilterGyroCoef(0.99);

    pitch_setpoint = 0.0;
    pitch_controller.setTimeStep(5);

    left_stepper.begin(27, 14);
    left_stepper.setAcceleration(20000);
    right_stepper.begin(25, 26);
    right_stepper.setAcceleration(20000);

    stepper_timer = timerBegin(0, 20, true);
    timerAttachInterrupt(stepper_timer, on_stepper_timer, false);
    timerAlarmWrite(stepper_timer, 50, true);
    timerAlarmEnable(stepper_timer);
}

void loop()
{
    static unsigned long loop_count = 0;
    static unsigned long last_print_time_us = micros();
    
    imu.update();
    pitch = -imu.getAngleY();
    pitch_controller.run();

    left_stepper.spin(pitch_action * abs(pitch_action));
    right_stepper.spin(pitch_action * abs(pitch_action));

    if (loop_count++ >= 1000) {
        auto now_us = micros();
        Serial.printf("pitch = %7.2lf, pitch_action = %7.1lf, sample_rate = %.1lf\n", pitch, pitch_action, (double)loop_count / (now_us - last_print_time_us) * 1000000.0);
        last_print_time_us = micros();
        loop_count = 0;
    }
}

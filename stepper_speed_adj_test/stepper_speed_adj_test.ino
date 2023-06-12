#include <Wire.h>
#include <MPU6050.h>
#include <imuFilter.h>
#include <AutoPID.h>

const unsigned IMU_SDA_PIN = 33;
const unsigned IMU_SCL_PIN = 32;
const unsigned LEFT_STEP_PIN = 27;
const unsigned LEFT_DIR_PIN = 14;
const unsigned RIGHT_STEP_PIN = 25;
const unsigned RIGHT_DIR_PIN = 26;

static double pitch, target_pitch, motor_speed;

MPU6050 imu;
imuFilter fusion;
AutoPID balance_controller(&pitch, &target_pitch, &motor_speed, -10000, 10000, 200, 50, 0);

void setup()
{
    Serial.begin(1000000);

    Wire.setPins(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);

    imu.initialize();

    int16_t ax, ay, az;
    imu.getAcceleration(&ax, &ay, &az);
    fusion.setup(ax, ay, az);

    target_pitch = -24.0;
    balance_controller.setTimeStep(10);

    pinMode(LEFT_STEP_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    pinMode(RIGHT_STEP_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
}

void loop()
{
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    fusion.update(gx, gy, gz, ax, ay, az);
    pitch = -atan2(ax, az) / 6.28 * 360;

//    balance_controller.run();
//    
//    bool forward = motor_speed > 0;
//    unsigned long step_interval_us = 1000000 / abs(motor_speed);
//    
//    static unsigned long last_step_time_us;
//    unsigned long now = micros();
//    if (now - last_step_time_us > step_interval_us) {
//        if (forward) {
//            digitalWrite(LEFT_DIR_PIN, HIGH);
//            digitalWrite(RIGHT_DIR_PIN, HIGH);
//        } else {
//            digitalWrite(LEFT_DIR_PIN, LOW);
//            digitalWrite(RIGHT_DIR_PIN, LOW);
//        }
//        digitalWrite(LEFT_STEP_PIN, HIGH);
//        digitalWrite(RIGHT_STEP_PIN, HIGH);
//        delayMicroseconds(1);
//        digitalWrite(LEFT_STEP_PIN, LOW);
//        digitalWrite(RIGHT_STEP_PIN, LOW);
//
//        last_step_time_us = now;
//    }

    Serial.printf("%6.1f, %6.1f, %6.1f\n", fusion.pitch(), fusion.yaw(), fusion.roll());
}

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <AutoPID.h>

#define PID_OUT_MIN -5000
#define PID_OUT_MAX 5000
#define KP 100
#define KI 30
#define KD 0

double angle, speed, target_angle;

MPU6050 accelgyro(0x68);
AutoPID motorPid(&angle, &target_angle, &speed, PID_OUT_MIN, PID_OUT_MAX, KP, KI, KD);

void setup() {
    Serial.begin(1000000);

    delay(500);
    
    Wire.setPins(33, 32);
    Wire.begin();
    Wire.setClock(400000);
    accelgyro.initialize();
    accelgyro.setXGyroOffset(80);
    accelgyro.setYGyroOffset(-38);
    accelgyro.setZGyroOffset(30);
    accelgyro.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_16);
    accelgyro.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_2000);
    accelgyro.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_10);

    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);
    pinMode(27, OUTPUT);
    pinMode(14, OUTPUT);

    target_angle = 22.0;
    motorPid.setTimeStep(3);

    printf("%hhi, %hhi, %hhi\n", accelgyro.getXGyroOffsetTC(), accelgyro.getYGyroOffsetTC(), accelgyro.getZGyroOffsetTC());
}

void loop() {
//    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
//    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
//    Serial.printf("ax:%6i ay:%6i az:%6i   gx:%6i gy:%6i gz:%6i   mx:%6i my:%6i mz:%6i\n", ax, ay, az, gx, gy, gz, mx, my, mz);

//    angle = atan((double)ax/az) / 6.28 * 360;
//    
//    motorPid.run();
//
//    Serial.printf("angle=%5.1lf steps_per_sec=%5.1lf\n", angle, speed);
//
//    bool forward = (speed < 0) ? true : false;
//    digitalWrite(26, forward);
//    digitalWrite(14, forward);
//    
//    long unsigned micros_per_step = 1000000.0f / abs(speed);
//    static long unsigned last_step_time = 0;
//    auto now = micros();
//    if (now - last_step_time > micros_per_step) {
//        digitalWrite(27, 1);
//        digitalWrite(25, 1);
//        delayMicroseconds(100);
//        digitalWrite(27, 0);
//        digitalWrite(25, 0);
//        delayMicroseconds(100);
//        last_step_time = now;
//    }
}

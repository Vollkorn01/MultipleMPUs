// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// 2016-05-14 github.com/eadf
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2016-05-14 - First revision

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012, 2016 Jeff Rowberg

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

#include "I2Cdev.h"
#include "MPU6050_Wrapper.h"
#include "TogglePin.h"
#include "DeathTimer.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


// uncomment to get (more) nicely formatted quaternions on console
//#define OUTPUT_READABLE_QUATERNION


// number of IMUs
#define N_IMU 5

MPU6050_Array mpus(N_IMU);

// MPU-6050 AD0 pins
static uint8_t mpu6050_ad0_pins[N_IMU] = {
  //0, 1,
  5, 6,
  /*9,*/ 10,
  11, 12, //13,
};

// MPU-6050 gyro offsets
static int8_t mpu6050_gyro_offsets[N_IMU][3] = {
  0
  // {220, 76, -85}, (example)
};

// MPU-6050 accel Z offsets
static int16_t mpu6050_accel_offsets[N_IMU] = {
  0
  // 1788, (example)
};


#define LED_PIN 13

#define OUTPUT_SERIAL Serial


uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


TogglePin activityLed(LED_PIN, 100);
DeathTimer deathTimer(5000L);


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // setup I2C
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(115200);

  while (!Serial)
    ; // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize IMUs
  for (int i = 0; i < N_IMU; i++) {
    Serial.print(F("Adding IMU "));
    Serial.println(i);
    mpus.add(mpu6050_ad0_pins[i]);
  }

  Serial.println(F("Initializing IMUs..."));
  mpus.initialize();

  // configure LED
  pinMode(LED_PIN, OUTPUT);

  // verify connection
  Serial.println(F("Testing IMU connections..."));
  if (mpus.testConnection()) {
    Serial.println(F("MPU6050 connection successful"));
  } else {
    mpus.halt(F("MPU6050 connection failed, halting"));
  }

  // wait for ready
  /*
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read())
    ; // empty buffer
  while (!Serial.available())
    activityLed.update(); // flash led while waiting for data
  while (Serial.available() && Serial.read())
    ; // empty buffer again
  */
  activityLed.setPeriod(500); // slow down led to 2Hz

  // initialize DMP
  Serial.println(F("Initializing DMP..."));
  mpus.dmpInitialize();

  // set offsets
  MPU6050_Wrapper *mpu;
  for (int i = 0; i < N_IMU; i++) {
    mpu = mpus.select(i);
    mpu->_mpu.setXGyroOffset(mpu6050_gyro_offsets[i][0]);
    mpu->_mpu.setYGyroOffset(mpu6050_gyro_offsets[i][1]);
    mpu->_mpu.setZGyroOffset(mpu6050_gyro_offsets[i][2]);
    mpu->_mpu.setZAccelOffset(mpu6050_accel_offsets[i]);
  }

  for (int i = 0; i < N_IMU; i++) {
    mpus.programDmp(i);
  }
}


void handleMPUevent(uint8_t mpu) {
  MPU6050_Wrapper *currentMPU = mpus.select(mpu);

  // reset interrupt flag and get INT_STATUS byte
  currentMPU->getIntStatus();

  // check for overflow (happens when code is too slow)
  if ((currentMPU->_mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT))
      || currentMPU->_fifoCount >= 1024) {
    // reset so we can continue cleanly
    currentMPU->resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  // check for DMP data ready interrupt
  if (currentMPU->_mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // read and dump a packet if the queue contains more than one
    while (currentMPU->_fifoCount >= 2 * currentMPU->_packetSize) {
      // read and dump one sample
      currentMPU->getFIFOBytes(fifoBuffer);
    }

    // read a packet from FIFO
    currentMPU->getFIFOBytes(fifoBuffer);
    // get quaternion data from packet
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    OUTPUT_SERIAL.print("quat:"); OUTPUT_SERIAL.print(mpu); OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(q.w);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(q.x);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(q.y);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.println(q.z);
#endif

    // display quaternion in correct format for MotionSuit script
    Serial.print(q.w, 2);
    Serial.print(",");
    Serial.print(q.x, 2);
    Serial.print(",");
    Serial.print(q.y, 2);
    Serial.print(",");
    Serial.print(q.z, 2);
    Serial.print(";");
  }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  MPU6050_Wrapper *mpu;
  for (int i = 0; i < N_IMU; i++) {
    mpu = mpus.select(i);
    if (mpu->isDue()) {
      handleMPUevent(i);
    }
  }

  activityLed.update();
  deathTimer.update();
}


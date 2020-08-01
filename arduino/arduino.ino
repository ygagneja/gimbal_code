#include "I2Cdev.h"
#include "Servo.h"
#include "SoftwareSerial.h"

SoftwareSerial myConnection(0, 1);
Servo myservo1;
Servo myservo2;
Servo myservo3;
int pos1 = 90;
int pos2 = 90;
int pos3 = 70;
int flag1 = 0;
int flag2 = 0;
int flag3 = 0;

int e1 = 0, e2 = 0, e3 = 0;

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    myservo1.attach(4);
    myservo2.attach(5);
    myservo3.attach(3);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    myConnection.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(150);
    mpu.setYGyroOffset(-44);
    mpu.setZGyroOffset(10);
    mpu.setZAccelOffset(847); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(10, INPUT);
    pinMode(11, INPUT);
    pinMode(12, INPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        myConnection.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            //Serial.print("euler\t");
            Serial.print((int)(euler[0] * 180/M_PI) + pos1);
            Serial.print("\t");
            //Serial.print((int)(euler[1] * 180/M_PI));
            //Serial.print("\t");
            //Serial.println((int)(euler[2] * 180/M_PI));
        #endif
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            Serial.print((int)(ypr[1] * 180/M_PI) + pos2);
            Serial.print("\t");
            Serial.print((int)(ypr[2] * 180/M_PI) + pos3);
            Serial.println("\t");
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    e1 = (int)(euler[0] * 180/M_PI);
    e2 = (int)(ypr[1] * 180/M_PI);
    e3 = (int)(ypr[2] * 180/M_PI);
    if(pos1+e1 < 180 && pos1+e1 > 0)
    {
      myservo1.write(pos1+e1);
    }
    if(pos2+e2 < 180 && pos2+e2 > 0)
    {
      myservo2.write(pos2+e2);
    }
    if(pos3+e3 < 180 && pos3+e3 > 0)
    {
      myservo3.write(pos3+e3);
    }
    
//    if (digitalRead(10) && flag1 == 0) {
//      pos1 += 1;
//      if(digitalRead(10) && pos1 == 180)
//        flag1 = 1;
//    }
//    if (digitalRead(10) && flag1 == 1) {
//      pos1 -= 1;
//      if(digitalRead(10) && pos1 == 0)
//        flag1 = 0;
//    }
//    if (digitalRead(11) && flag2 == 0) {
//      pos2 += 1;
//      if(digitalRead(11) && pos2 == 180)
//        flag2 = 1;
//    }
//    if (digitalRead(11) && flag2 == 1) {
//      pos2 -= 1;
//      if(digitalRead(11) && pos2 == 0)
//        flag2 = 0;
//    }
//    if (digitalRead(12) && flag3 == 0) {
//      pos3 += 1;
//      if(digitalRead(12) && pos3 == 180)
//        flag3 = 1;
//    }
//    if (digitalRead(12) && flag3 == 1) {
//      pos3 -= 1;
//      if(digitalRead(12) && pos3 == 0)
//        flag3 = 0;
//    }
    
    delay(15);
    
}

#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID tuning values (adjust as necessary)
double setpoint = 188; // Adjust to your bot's upright position
double Kp = 21;
double Kd = 0.9;
double Ki = 140;

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false; // Indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

void setup() {
    Serial.begin(115200);

    // Initialize MPU6050
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // Verify connection
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // Load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // Supply gyro offsets (adjust as needed)
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688);

    if (devStatus == 0) {
        // Turn on DMP
        Serial.println("Enabling DMP...");
        mpu.setDMPEnabled(true);

        // Enable interrupt detection
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set DMP ready flag
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        // Initialize PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);

    } else {
        // Handle DMP initialization failure
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
        while (1); // Stop execution
    }

    // Initialize motor pins
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);

    // Stop motors initially
    analogWrite(6, 0);
    analogWrite(9, 0);
    analogWrite(10, 0);
    analogWrite(11, 0);
}

void loop() {
    if (!dmpReady) return;

    // Wait for MPU interrupt or available packet
    while (!mpuInterrupt && fifoCount < packetSize);

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Handle FIFO overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
        return;
    }

    // Read DMP data if available
    while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        input = ypr[1] * 180 / M_PI + 180;

        // Perform PID computation
        pid.Compute();

        // Drive motors based on PID output
        if (output > 0) {
            Forward(output);
        } else {
            Reverse(-output);
        }

        // Print debug information (optional)
        Serial.print("Input: ");
        Serial.print(input);
        Serial.print(" Output: ");
        Serial.println(output);
    }
}

void Forward(double pwm) {
    analogWrite(6, pwm);
    analogWrite(9, 0);
    analogWrite(10, pwm);
    analogWrite(11, 0);
}

void Reverse(double pwm) {
    analogWrite(6, 0);
    analogWrite(9, pwm);
    analogWrite(10, 0);
    analogWrite(11, pwm);
}

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>
#include "servo_control.h"
#include "Servo_Calibration.h"
#include "PID_controller.h"
#include "timer.h"

// Global variables
float Yaw, Pitch, Roll;
MPU6050 mpu;
int16_t accelX, accelY, accelZ;  // Use int16_t for raw sensor values
float altitude, Vz=0, az;
unsigned long start_time, end_time;
float lastVz = 0.0;
unsigned long lastTime = 0;
unsigned long lastTimea = 0;

#define LED_PIN 13 // 

// Supply your own gyro offsets here, scaled for min sensitivity
int MPUOffsets[6] = {-5047, -2670, 543, 234, -20, -36};

// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // Join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // Indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive = 0; // Tests if the interrupt is triggering
int IsAlive = -20; // Counts interrupt start at -20 to get 20+ good values before assuming connected

// MPU control/status vars
uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z] quaternion container
VectorInt16 aa;         // [x, y, z] accel sensor measurements
VectorInt16 aaReal;     // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z] gravity vector
float euler[3];         // [psi, theta, phi] Euler angle container
float ypr[3];           // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
byte StartUP = 100; // Lets get 100 readings from the MPU before we start trusting them

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // Initialize device
  mpu.initialize();
  // Load and configure the DMP
  devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "Initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; // Only try 10 times
    delay(1000);
    MPU6050Connect(); // Let's try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // Enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  attachInterrupt(0, dmpDataReady, RISING); // Pin 2 on the Uno
  mpuIntStatus = mpu.getIntStatus();
  // Get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it stabilize
  mpu.resetFIFO(); // Clear FIFO buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // Wait for next interrupt
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP() {
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // We have failed, reset and wait till next time!
    digitalWrite(LED_PIN, LOW); // Let's turn off the blinking light so we can see we are failing.
    mpu.resetFIFO(); // Clear the buffer and start over
  } else {
    while (fifoCount >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // Let's do the magic and get the data
      fifoCount -= packetSize;
    }
    LastGoodPacketTime = millis();
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}

// ================================================================
// ===                        MPU Math                          ===
// ================================================================
void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Yaw = (ypr[0] * 180.0 / M_PI);
  Pitch = (ypr[1] * 180.0 / M_PI);
  Roll = (ypr[2] * 180.0 / M_PI);
}

// ================================================================
// ===                      MPU Physics                         ===
// ================================================================
// Global variable to store the offset (calibrate this when stationary)

float az_offset = 1.001462;  // Use the calibrated offset

float calculate_az() {
    // Read raw accelerometer data
    mpu.getMotion6(&accelX, &accelY, &accelZ, nullptr, nullptr, nullptr);

    // Apply offset correction
    accelX -= MPUOffsets[0];
    accelY -= MPUOffsets[1];
    accelZ -= MPUOffsets[2];

    // Convert raw values to acceleration in g
    float Ax = accelX / 16384.0;
    float Ay = accelY / 16384.0;
    float Az = accelZ / 16384.0;

    // Convert Pitch and Roll to radians
    float pitchRad = Pitch * M_PI / 180.0;
    float rollRad = Roll * M_PI / 180.0;

    // Calculate the acceleration in the inertial Z-axis (earth frame)
    float accZ_inertial = Az * cos(pitchRad) * cos(rollRad) +
                          Ay * sin(rollRad) +
                          Ax * sin(pitchRad);

    // Apply the calibrated offset to the calculated acceleration
    accZ_inertial -= az_offset;

    // Apply a small threshold to ignore noise and small biases in az
    float threshold = 0.08; // Threshold in m/s^2
    if (accZ_inertial > -threshold && accZ_inertial < threshold) {
        accZ_inertial = 0;
    }

    return accZ_inertial;
}




float calculate_Vz(float az) {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Convert ms to seconds

  Vz=az*dt+Vz;
  lastVz = Vz;
  if(az==0){
    Vz=0;
  }
  lastTime = currentTime;
  return Vz;
}







float last_altitude=0;
float calculate_altitude(float Vz,float lastVz) {
   unsigned long currentTimea = millis();
   float dt2 = (currentTimea - lastTimea) / 1000.0; // Convert ms to seconds

  altitude=0.5*az*dt2*dt2+Vz*dt2+last_altitude;
  last_altitude=altitude;
  lastTimea = currentTimea;
  return altitude;
}

// ================================================================
// ===                        Setup                             ===
// ================================================================
bool initial = false;
void setup() {
  start_time = micros();
  Serial.begin(9600); //115200
  Serial.flush();
  while (!Serial);
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
  pinMode(LED_PIN, OUTPUT);
  pitchservo.attach(9);
  rollservo.attach(10);
  pitchservo.write(pitch_mid);
  rollservo.write(roll_mid);
  time(start_delay);
  while (Pitch == 0 && Roll == 0) {
    GetDMP();
    Serial.print("Pitch: "); Serial.println(Pitch);
    Serial.print("Roll: "); Serial.println(Roll);
  }
  pitchinit = Pitch;
  rollinit = Roll;
  Serial.print("Pitch initial: "); Serial.println(pitchinit);
  Serial.print("Roll initial: "); Serial.println(rollinit);
  end_time = micros();
  iteration_time = end_time - start_time;
}

// ================================================================
// ===                        Loop                              ===
// ================================================================
void loop() {
  start_time = micros();
  GetDMP();
  pitcherror = error(pitchinit, Pitch);  // Getting the initial errors for pitch
  rollerror = error(rollinit, Roll);     // Getting the initial errors for roll

  PID_pitch = PID(pitcherror, &pitchintegralgain, &previouspitcherror);  // PID for pitch
  PID_roll = PID(rollerror, &rollintegralgain, &previousrollerror);      // PID for roll

  altitude = calculate_altitude(Vz,lastVz);
  az = calculate_az();
  Vz = calculate_Vz(az);
  
  servo_write(PID_pitch_2(PID_pitch), PID_roll_1(PID_roll));  // Give the servos the values to write 
  
  Serial.print(az); Serial.print(",");
  Serial.print(altitude); Serial.print(",");
  Serial.print(Vz); Serial.print(",");
  Serial.print(PID_pitch); Serial.print(",");
  Serial.print(PID_roll); Serial.print(",");
  Serial.print(Yaw); Serial.print(",");
  Serial.print(Pitch); Serial.print(",");
  Serial.println(Roll);
  end_time = micros();
  iteration_time = end_time - start_time;
}
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>
#include "servo_control.h"
#include "Servo_Calibration.h"
#include "PID_controller.h"
#include "timer.h"
#include <SD.h>
#include <SPI.h>

float Yaw, Pitch, Roll;
MPU6050 mpu;
unsigned long start_time;
unsigned long end_time;

#define DEBUG
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define  DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     
#define DPRINTLN(...)      
#endif

#define LED_PIN 13 // 
#define SD_CS_PIN 4 // Chip select pin for SD card

// Supply your own gyro offsets here, scaled for min sensitivity use MPU6050_calibration.ino
int MPUOffsets[6] = {-3185, -1032, 1358, -40, -11, 60};

const int numReadings = 10; // Reduce the number of readings to fit in memory and allow for buffer
struct SensorData {
    float roll, pitch;
    float rollPID, pitchPID;
};

SensorData readings[numReadings];
int currentReading = 0;

// Function to initialize SD card
void initSD() {
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
    } else {
        Serial.println("SD card initialized.");
        // Open the file in write mode to add the header
        File dataFile = SD.open("datalog.csv", FILE_WRITE);
        if (dataFile) {
            dataFile.println("Roll,Pitch,RollPID,PitchPID"); // CSV header
            dataFile.close();
        }
    }
}

// Function to store data in arrays
void storeData(float roll, float pitch, float rollPID, float pitchPID) {
    if (currentReading < numReadings) {
        readings[currentReading] = {roll, pitch, rollPID, pitchPID};
        currentReading++;
    }
}

// Function to write buffered data to SD card
void writeBufferToSD() {
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
        for (int i = 0; i < currentReading; i++) {
            dataFile.print(readings[i].roll);
            dataFile.print(",");
            dataFile.print(readings[i].pitch);
            dataFile.print(",");
            dataFile.print(readings[i].rollPID);
            dataFile.print(",");
            dataFile.println(readings[i].pitchPID);
        }
        dataFile.close();
        Serial.println("Buffer written to SD card.");
        currentReading = 0; // Reset buffer
    } else {
        Serial.println("Error opening datalog.csv");
    }
}

void i2cSetup() {
  // Join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

volatile bool mpuInterrupt = false;     // Indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

int FifoAlive = 0; // Tests if the interrupt is triggering
int IsAlive = -20; // Counts interrupt start at -20 to get 20+ good values before assuming connected
uint8_t mpuIntStatus; // Holds actual interrupt status byte from MPU
uint8_t devStatus; // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    char * StatStr[5] = { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};
    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; // Only try 10 times
    delay(1000);
    MPU6050Connect();
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
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  "); Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, RISING); // Pin 2 on the Uno
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000);
  mpu.resetFIFO();
  mpu.getIntStatus();
  mpuInterrupt = false;
}

void GetDMP() {
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) {
    digitalWrite(LED_PIN, LOW);
    mpu.resetFIFO();
  } else {
    while (fifoCount  >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }
    LastGoodPacketTime = millis();
    MPUMath();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Yaw = (ypr[0] * 180.0 / M_PI);
  Pitch = (ypr[1] *  180.0 / M_PI);
  Roll = (ypr[2] *  180.0 / M_PI);
}

bool initial = false;

void setup() {
  start_time = micros();
  Serial.begin(115200);
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

  // Initialize SD card
  initSD();

  while(Pitch == 0 && Roll == 0) {
    GetDMP();
    Serial.print("pitch : "); Serial.println(Pitch);
    Serial.print("Roll : "); Serial.println(Roll);
  }
  pitchinit = Pitch;
  rollinit = Roll;
  Serial.print("pitch : "); Serial.println(Pitch);
  Serial.print("Roll : "); Serial.println(Roll);
  Serial.print("pitch initial: "); Serial.println(pitchinit);
  Serial.print("roll initial: "); Serial.println(rollinit);
  end_time = micros();
  iteration_time = end_time - start_time;
}

void loop() {
  start_time = micros();
  GetDMP();
  pitcherror = error(pitchinit, Pitch);
  rollerror = error(rollinit, Roll);
  PID_pitch = PID(pitcherror, &pitchintegralgain, &previouspitcherror);
  PID_roll = PID(rollerror, &rollintegralgain, &previousrollerror);
  servo_write(PID_pitch_2(PID_pitch), PID_roll_1(PID_roll));

  // Store data in buffer
  storeData(Roll, Pitch, PID_roll, PID_pitch);

  // Write buffer to SD card periodically or when buffer is full
  if (currentReading >= numReadings) {
    writeBufferToSD();
  }

  end_time = micros();
  iteration_time = end_time - start_time;
  Serial.print("Iteration Time");Serial.println(iteration_time);
}

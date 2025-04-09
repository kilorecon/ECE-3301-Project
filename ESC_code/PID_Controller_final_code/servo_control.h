#ifndef servo_control
#define servo_control
#include "Wire.h"
#include <Servo.h>
#include "PID_controller.h"
// ================================================================
// ===                Controlling the servo                     ===
// ================================================================ 
Servo pitchservo;
Servo rollservo;
//writing onto the servos using he function
void servo_write(float pitchservoangle, float rollservoangle){
  pitchservo.write(pitchservoangle);
  rollservo.write(rollservoangle);
  //delay(5);
}
#endif 
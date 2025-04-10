#ifndef PID_controller
#define PID_controller

// ================================================================
// ===                         P.I.D                            ===
// ================================================================ 
// To use this function, you need to feed it the initial error that occurred in the MPU, 
// and create 2 separate variables which are the old error and the integral gain 
// to feed back into the loop to have a functioning PID.

float gain_mult = 0.5;
float derivative_mult = 40;
float integral_mult = 0;
float iteration_time; // You may want to calculate this dynamically

// Defining all the errors and the gains and setting them to 0
float pitcherror = 0, pitchintegralgain = 0, previouspitcherror = 0;
float rollerror = 0, rollintegralgain = 0, previousrollerror = 0;
float pitchinit, rollinit; // the initial values of error so we could get the error for the PID
float PID_roll, PID_pitch; // the values that are spit out by the PID that we will use with the calibration functions

// PID function that gives out the angle for the servo to work
float PID(float initerror, float* integralgain, float* olderror) {
  float gain, derivativegain, totalgain;
  gain = initerror * gain_mult; // proportional term
  derivativegain = ((initerror - *olderror) / iteration_time) * derivative_mult; // derivative term
  *integralgain = (*integralgain + initerror) * integral_mult; // integral term
  
  // Optional: Limit the integral term to prevent windup
  float max_integral = 10.0; // Adjust as necessary
  if (*integralgain > max_integral) *integralgain = max_integral;
  if (*integralgain < -max_integral) *integralgain = -max_integral;
  
  totalgain = -(gain + derivativegain + *integralgain); // sum of all terms
  *olderror = initerror;
  return totalgain;
}

// Function to calculate the error that was present at that iteration and later feed it to the PID
float error(float init, float current) {
  return current - init;
}

#endif
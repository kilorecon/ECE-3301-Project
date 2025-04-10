#ifndef PID_controller
#define PID_controller
#include "MPU6050_6Axis_MotionApps20.h"
#include "Quaternion_Math.h"

// ================================================================
// ===                         P.I.D                            ===
// ================================================================ 
// To use this function, you need to feed it the initial error that occurred in the MPU, 
// and create 2 separate variables which are the old error and the integral gain 
// to feed back into the loop to have a functioning PID.

float gain_mult = 7.0;
float derivative_mult = 0.2;
float integral_mult = 0;
float iteration_time; // You may want to calculate this dynamically

// Defining all the errors and the gains and setting them to 0
float pitcherror = 0, pitchintegralgain = 0, previouspitcherror = 0;
float rollerror = 0, rollintegralgain = 0, previousrollerror = 0;
float pitchinit, rollinit; // the initial values of error so we could get the error for the PID
float PID_roll, PID_pitch; // the values that are spit out by the PID that we will use with the calibration functions
Quaternion qError, qinit, PID_q, qintegralgain, previousqerror; 

// PID function that gives out the angle for the servo to work

// Quaternion-based PID controller function
Quaternion PID(Quaternion qError, Quaternion* integralgain, Quaternion* olderror) {
    // Use qError to calculate the rotational velocity needed for correction
    Quaternion gain = qError * gain_mult; //scalar_multiply(qError, gain_mult);  // Proportional term (adjust gain accordingly)
    *integralgain = *integralgain + qError * integral_mult;      // Integral term (adjust gain accordingly)
    Quaternion derivativegain = (qError + -*olderror) * derivative_mult;  // Derivative term (adjust gain accordingly)
    *olderror = qError;
  
    // Optional: Limit the integral term to prevent windup
    /*float max_integral = 5.0; // Adjust as necessary
    if (integralgain->w > max_integral) integralgain->w = max_integral;
    if (integralgain->x > max_integral) integralgain->x = max_integral;
    if (integralgain->y > max_integral) integralgain->y = max_integral;
    if (integralgain->z > max_integral) integralgain->z = max_integral;*/

    return (-gain + -*integralgain + -derivativegain);  // Return the PID control output
}

// Function to compute quaternion error: qError = qInit^-1 * q
Quaternion quaternionError(Quaternion target, Quaternion current) {
    Quaternion qInv = target.getConjugate();  // Get the inverse (conjugate) of the target quaternion
    Quaternion qError = qInv * current;  // qError = qInit^-1 * q
    return qError;
}
#endif
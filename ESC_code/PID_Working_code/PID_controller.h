#ifndef PID_controller
#define PID_controller
// ================================================================
// ===                         P.I.D                            ===
// ================================================================ 
//to use this function you ave to feed it the initial error that occured in the mpu, and create 2 seperate vairables which is the old 
//error and the integral gain to feed back into the loop to have a functionning PID
//the PID code that is used
//these are the values that are multiplied to tune the PID
float gain_mult=0.75;
float derivative_mult=0.3;
float integral_mult=0.01;
float iteration_time=17;
//this is the PID function that gives out the angle for the servo to work

//Defining all the errors and the gains and setting them to 0
float pitcherror=0,pitchintegralgain=0,previouspitcherror=0;
float rollerror=0,rollintegralgain=0,previousrollerror=0;
float pitchinit,rollinit;//the inital values of error so we could get the error for the PID
float PID_roll,PID_pitch;//the values that are spit out by the PID that we will use with the calibration functions


float PID(float initerror,float* integralgain,float* olderror){
  float gain,derivativegain,totalgain;
  gain=initerror*gain_mult;                                 //initial error part
  derivativegain=((initerror-*olderror)/iteration_time)*derivative_mult;//derivative part
  *integralgain=(*integralgain+(initerror))*integral_mult;    //integral gain
  totalgain=-(gain+derivativegain+*integralgain);           //addition of all the errors
  *olderror=initerror;
  //Serial.print("integralgain:");Serial.println(*integralgain);
  Serial.print("this is total gain: ");Serial.println(totalgain);
  return totalgain;
}
//function to calculate the error that was present at that interation and later feed it to the PID
float error (float init,float current){
  float error;
  error=current-init;
  Serial.print("this is the error: ");Serial.println(error);
  return error;
}
#endif 
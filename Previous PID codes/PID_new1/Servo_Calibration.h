#ifndef Servo_Calibration
#define Servo_Calibration
// ================================================================
// ===            Calibrtion of the servo above                 ===
// ================================================================ 
//these functions will be used for the servo that is on the top half of the gimbal as the values that it has to be given are different
//setting the bounds of the servo in which the movement occurs
//roll  pin 10
float roll_min=0;       //the minimum the servo can move
float roll_mid=12;      //the midpoint where the motor holder is centered
float roll_max=18;      //the max the servo can move
//pitch pin 9
float pitch_min=0;      //the minimum the servo can move
float pitch_mid=31;     //the midpoint where the motor holder is centered
float pitch_max=43;     //the max the servo can move
//bufferr in order to determine if it passed through the previous if statement
int Buffer=0;
// ================================================================
// ===                        Pitch                             ===
// ===                    READ BEFORE USE                       ===
// ================================================================ 




//this function is used if while looking directly at the servo, the min value is close to the servo and the max value is away from the servo 
//also the negative values of the mpu when tilted should be on the servo side and the positive on the right side
//to test how it works put this code into a C++ compiler and change the value of PID_pitch_value


//#include <iostream>
// using namespace std;
// float PID_pitch_value=-23;
// float pitch_min=0;
// float pitch_mid=20;
// float pitch_max=40;
// float PID_pitch_1(float PID_pitch_value){
//   if (PID_pitch_value<(pitch_min-pitch_mid)){
//     PID_pitch_value=pitch_min-pitch_mid;
//   }
//   if (PID_pitch_value>pitch_max-pitch_mid){
//     PID_pitch_value=pitch_max-pitch_mid;
//   }
//   PID_pitch_value=PID_pitch_value+pitch_mid;
//   cout<<PID_pitch_value;
//   return PID_pitch_value;
// }
// int main() {
//     PID_pitch_1(PID_pitch_value);
//     return 0;
// }


float PID_pitch_1(float PID_pitch_value){
  if (PID_pitch_value<(pitch_min-pitch_mid)){ //setting the value coming from the PID to the min value the servo could move 
    PID_pitch_value=pitch_min-pitch_mid;
  }
  if (PID_pitch_value>pitch_max-pitch_mid){ //setting the value coming from the PID to the max value the servo could move
    PID_pitch_value=pitch_max-pitch_mid;
  }
  PID_pitch_value=PID_pitch_value+pitch_mid;//centering the motor holder at the mid value of the servo 
  return PID_pitch_value;                   //returnig the value so the servo could move it
}

//this function is used if while looking directly at the servo, the min value is away from the servo and the max value is close to the servo 
//also the negative values of the mpu when tilted should be on the servo side and the positive on the right side

float PID_pitch_2(float PID_pitch_value){
  int buffer=0;
  if (PID_pitch_value<pitch_mid-pitch_max){   //setting the max value that the servo can travel to the left 
    PID_pitch_value=pitch_mid-pitch_max;
  }
  if (PID_pitch_value>pitch_mid-pitch_min){    //setting the max value that the servo can travel to the right
    PID_pitch_value=pitch_mid-pitch_min;
  }
  if (PID_pitch_value>0){                      // making sure to switch the bounds
    PID_pitch_value=pitch_mid-PID_pitch_value;
    buffer++; //making sure that if it passed by this and exceeded the max value it does not automatically go to 0 on the next if statement
  }
  if (PID_pitch_value==0 && buffer==0){        //making sure that the 0 value is covered
    PID_pitch_value=pitch_mid;
  }
  if (PID_pitch_value<0){                      //also switching the bounds on the other side
    PID_pitch_value=-(PID_pitch_value-pitch_mid);
  }
  return PID_pitch_value;                      //returning the function to the value that called it
}

//you can plug this code in a C++ compiler in order to know how the code works and executes, just switch the PID_pitch_value parameter

// #include <iostream>
// using namespace std;
// float PID_pitch_value=19;
// float pitch_min=0;
// float pitch_mid=20;
// float pitch_max=40;
// int buffer=0;
// float PID_pitch_2(float PID_pitch_value,int buffer){
//   if (PID_pitch_value<pitch_mid-pitch_max){
//     PID_pitch_value=pitch_mid-pitch_max;
//   }
//   if (PID_pitch_value>pitch_mid-pitch_min){
//     PID_pitch_value=pitch_mid-pitch_min;
//   }
//   if (PID_pitch_value>0){
//     PID_pitch_value=pitch_mid-PID_pitch_value;
//     buffer++;
//   }
//   if (PID_pitch_value==0 && buffer==0){
//     PID_pitch_value=pitch_mid;
//   }
//   if (PID_pitch_value<0){
//     PID_pitch_value=-(PID_pitch_value-pitch_mid);
//   }
//   cout<<PID_pitch_value;
//   return PID_pitch_value;
// }
// int main() {
//     PID_pitch_2(PID_pitch_value,buffer);
//     return 0;
// }
// ================================================================
// ===                        Roll                             ===
// ===                    READ BEFORE USE                       ===
// ================================================================ 
//this function is used if while looking directly at the servo, the min value is close to the servo and the max value is away from the servo 
//also the negative values of the mpu when tilted should be on the servo side and the positive on the right side
//these have the same logic as the functions above
float PID_roll_1(float PID_roll_value){
  if (PID_roll_value<(roll_min-roll_mid)){ //setting the value coming from the PID to the min value the servo could move 
    PID_roll_value=roll_min-roll_mid;
  }
  if (PID_roll_value>roll_max-roll_mid){ //setting the value coming from the PID to the max value the servo could move
    PID_roll_value=roll_max-roll_mid;
  }
  PID_roll_value=PID_roll_value+roll_mid;//centering the motor holder at the mid value of the servo 
  return PID_roll_value;                   //returnig the value so the servo could move it
}

//this function is used if while looking directly at the servo, the min value is away from the servo and the max value is close to the servo 
//also the negative values of the mpu when tilted should be on the servo side and the positive on the right side

float PID_roll_2(float PID_roll_value){
  int buffer=0;
  if (PID_roll_value<roll_mid-roll_max){   //setting the max value that the servo can travel to the left 
    PID_roll_value=roll_mid-roll_max;
  }
  if (PID_roll_value>roll_mid-roll_min){    //setting the max value that the servo can travel to the right
    PID_roll_value=roll_mid-roll_min;
  }
  if (PID_roll_value>0){                      // making sure to switch the bounds
    PID_roll_value=roll_mid-PID_roll_value;
    buffer++; //making sure that if it passed by this and exceeded the max value it does not automatically go to 0 on the next if statement
  }
  if (PID_roll_value==0 && buffer==0){        //making sure that the 0 value is covered
    PID_roll_value=roll_mid;
  }
  if (PID_roll_value<0){                      //also switching the bounds on the other side
    PID_roll_value=-(PID_roll_value-roll_mid);
  }
  return PID_roll_value;                      //returning the function to the value that called it
}
#endif 
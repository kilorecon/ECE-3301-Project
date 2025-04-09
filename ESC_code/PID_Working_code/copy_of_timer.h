#ifndef timer
#define timer
//to change the delay of the star time you need to change the 'start_delay' value
int start_delay=10000;
//delaying the initial recording of values using a fucntion and delay
void time(int delay_1){
  int target=0;
  while (delay_1!=target){
    target=target+100;
    delay(100);
  }
}
#endif 

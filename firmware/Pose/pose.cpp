#include <pose.h>

Pose::Pose(){}

void Pose::squeezing_close_hand(int* targetAngles, float* totalTime, float* startTime){
  targetAngles[1] = -10;
  totalTime[1] = 1.0;
  startTime[1] = millis() / 1000.0;
  targetAngles[2] = 15;
  totalTime[2] = 1.0;
  startTime[2] = millis() / 1000.0;
  targetAngles[4] = -15;
  totalTime[4] = 1.0;
  startTime[4] = millis() / 1000.0;
  targetAngles[3] = 10;
  totalTime[3] = 1.0;
  startTime[3] = millis() / 1000.0;
  targetAngles[0] = 0;
  totalTime[0] = 1.0;
  startTime[0] = millis() / 1000.0;
  targetAngles[5] = 0;
  totalTime[5] = 1.0;
  startTime[5] = millis() / 1000.0;            
}

void Pose::squeezing_open_hand(int* targetAngles, float* totalTime, float* startTime){
  targetAngles[1] = 60;
  totalTime[1] = 1.0;
  startTime[1] = millis() / 1000.0;
  targetAngles[2] = -60;
  totalTime[2] = 1.0;
  startTime[2] = millis() / 1000.0;
  targetAngles[4] = 60;
  totalTime[4] = 1.0;
  startTime[4] = millis() / 1000.0;
  targetAngles[3] = -60;
  totalTime[3] = 1.0;
  startTime[3] = millis() / 1000.0;
  targetAngles[0] = -60;
  totalTime[0] = 1.0;
  startTime[0] = millis() / 1000.0;
  targetAngles[5] = -60;
  totalTime[5] = 1.0;
  startTime[5] = millis() / 1000.0;  
}

void Pose::waving_initial_arm(int* targetAngles, float* totalTime, float* startTime){
  targetAngles[7] = 7500;
  totalTime[7] = 2.0;
  startTime[7] = millis() / 1000.0;
  targetAngles[8] = 7500;
  totalTime[8] = 2.0;
  startTime[8] = millis() / 1000.0;
  targetAngles[9] = 7500;
  totalTime[9] = 2.0;
  startTime[9] = millis() / 1000.0;  
  targetAngles[10] = 4833;
  totalTime[10] = 2.0;
  startTime[10] = millis() / 1000.0;  
}

void Pose::waving_left(int* targetAngles, float* totalTime, float* startTime){
  targetAngles[7] = 8800;
  totalTime[7] = 1.5;
  startTime[7] = millis() / 1000.0;  
}

void Pose::waving_right(int* targetAngles, float* totalTime, float* startTime){
  targetAngles[7] = 6200;
  totalTime[7] = 1.5;
  startTime[7] = millis() / 1000.0;    
}

void Pose::waving_straight(int* targetAngles, float* totalTime, float* startTime){
  targetAngles[7] = 7500;
  totalTime[7] = 1.0;
  startTime[7] = millis() / 1000.0;      
}

void Pose::beckoning_initial_arm(int* targetAngles, float* totalTime, float* startTime){
  targetAngles[7] = 7500;
  totalTime[7] = 5.0;
  startTime[7] = millis() / 1000.0;
  targetAngles[8] = 7800;
  totalTime[8] = 5.0;
  startTime[8] = millis() / 1000.0;
  targetAngles[9] = 3500;
  totalTime[9] = 5.0;
  startTime[9] = millis() / 1000.0;  
  targetAngles[10] = 5100;
  totalTime[10] = 5.0;
  startTime[10] = millis() / 1000.0;  
}

void Pose::beckoning_inside(int* targetAngles, float* totalTime, float* startTime){
  targetAngles[8] = 7800;
  totalTime[8] = 1.0;
  startTime[8] = millis() / 1000.0;
}

void Pose::beckoning_outside(int* targetAngles, float* totalTime, float* startTime){
  targetAngles[8] = 6500;
  totalTime[8] = 1.0;
  startTime[8] = millis() / 1000.0;
}

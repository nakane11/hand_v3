#ifndef POSE_H
#define POSE_H

#include <Arduino.h>

class Pose {
 public:
  Pose();
  void squeezing_close_hand(int* targetAngles, float* totalTime, float* startTime);
  void squeezing_open_hand(int* targetAngles, float* totalTime, float* startTime);
  void waving_initial_arm(int* targetAngles, float* totalTime, float* startTime);
  void waving_left(int* targetAngles, float* totalTime, float* startTime);
  void waving_right(int* targetAngles, float* totalTime, float* startTime);
  void waving_straight(int* targetAngles, float* totalTime, float* startTime);
  void beckoning_initial_arm(int* targetAngles, float* totalTime, float* startTime); 
  void beckoning_inside(int* targetAngles, float* totalTime, float* startTime);
  void beckoning_outside(int* targetAngles, float* totalTime, float* startTime);  
};

#endif

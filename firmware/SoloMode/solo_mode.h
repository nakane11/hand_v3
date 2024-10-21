#ifndef SOLO_MODE_H
#define SOLO_MODE_H

#include <Arduino.h>
#include <mode.h>
#include <pose.h>
#include <cstdlib> // for rand() and srand()
#include <ctime>   // for time()

enum SoloModeTask {
                   STAYING,
                   SQUEEZING,
                   WAVING,
                   BECKONING,
                   TAPPING
};

class SoloMode : public Mode {
public:
  SoloMode();
  std::string updateMode(uint8_t* force_data) override;

  void processTask(int* targetAngles, float* totalTime, float* startTime, bool& updateCompleteFlag) override;
  std::string getName() override;

 private:
  SoloModeTask setNewTask(SoloModeTask previous_task);
  unsigned long getRandomValue(unsigned long minValue, unsigned long maxValue);

  SoloModeTask currentTask;
  unsigned long duration; // タスクの継続時間
  unsigned long previousMillis = 0; // 前回のミリ秒を保持
  unsigned long count;
  unsigned long total_count;
  Pose pose;
};

#endif

#include <solo_mode.h>

SoloMode::SoloMode(){
    srand(static_cast<unsigned>(time(0)));
    currentTask=setNewTask(SQUEEZING);
}

unsigned long SoloMode::getRandomValue(unsigned long minValue, unsigned long maxValue) {
    return rand() % (maxValue - minValue + 1) + minValue;
}

SoloModeTask SoloMode::setNewTask(SoloModeTask previous_task) {
  SoloModeTask new_task;
  if(previous_task!=STAYING){
    new_task = STAYING;
  }else{
    int randomIndex = rand() % 2 + 1; // 1か2を生成
    // new_task = static_cast<SoloModeTask>(randomIndex);
    new_task = SQUEEZING;
  }
  if(new_task==STAYING){
    previousMillis = millis(); // 経過時間のリセット
    duration = getRandomValue(5000, 15000); // 新しいdurationを設定
  }else if(new_task==SQUEEZING){
    // move arm
    count = 2*getRandomValue(1,3)+1;
    M5.Lcd.setCursor(0,20);
    M5.Lcd.println(count);     
  }else if(new_task==TAPPING){
    //move arm
    count = getRandomValue(3,15)+1;
  }
  M5.Lcd.setCursor(0,0);
  M5.Lcd.println(new_task);
  return new_task;
}

void SoloMode::processTask(int* targetAngles, float* totalTime, float* startTime, bool& updateCompleteFlag) {
  if(currentTask==STAYING){
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= duration){
      currentTask=setNewTask(currentTask);
    }
  }else if(currentTask==SQUEEZING){
    M5.Lcd.setCursor(0,20);
    M5.Lcd.println(count);    
    if(updateCompleteFlag){
      count--;   
      if(count==0){
        currentTask=setNewTask(currentTask);
      }else{
        // next target 偶数なら閉じる，奇数なら開く
        if(count%2==0){
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
        }else if(count%2==1){
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
        updateCompleteFlag = false;
      }
    }
  }else if(currentTask==TAPPING){
    //send angle vector
    count--;
    if(count==0){
      currentTask=setNewTask(currentTask);
    }
  }  
}

std::string SoloMode::getName() {
  return "SoloMode";
}

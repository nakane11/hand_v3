#include <solo_mode.h>

SoloMode::SoloMode(){
  srand(static_cast<unsigned>(time(0)));
  currentTask=setNewTask(SQUEEZING);
}

std::string SoloMode::updateMode(uint8_t* force_data) {
  return "SoloMode";  // モードを変えない
}

unsigned long SoloMode::getRandomValue(unsigned long minValue, unsigned long maxValue) {
    return rand() % (maxValue - minValue + 1) + minValue;
}

SoloModeTask SoloMode::setNewTask(SoloModeTask previous_task) {
  SoloModeTask new_task;
  if(previous_task!=STAYING){
    new_task = STAYING;
  }else{
    int randomIndex = rand() % 3 + 1; // 1か2を生成
    new_task = static_cast<SoloModeTask>(randomIndex);
  }

  if(new_task==STAYING){
    previousMillis = millis(); // 経過時間のリセット
    duration = getRandomValue(5000, 15000); // 新しいdurationを設定
  }else if(new_task==SQUEEZING){
    // move arm
    count = 2*getRandomValue(1,3)+1;
    M5.Lcd.setCursor(0,20);
    M5.Lcd.println(count);     
  // }else if(new_task==TAPPING){
  //   //move arm
  //   count = getRandomValue(3,15)+1;
  }else if(new_task==WAVING){
    //腕の移動+往復+初期位置に戻る
    total_count = 2*getRandomValue(2,4)+2;
    count = 0;
    M5.Lcd.setCursor(0,20);
    M5.Lcd.println(total_count);
  }else if(new_task==BECKONING){
    //腕の移動+往復
    total_count = 2*getRandomValue(2,3)+2;
    count = 0;
    M5.Lcd.setCursor(0,20);
    M5.Lcd.println(total_count);
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
        //偶数なら閉じる，奇数なら開く
        if(count%2==0){
          pose.squeezing_close_hand(targetAngles, totalTime, startTime);
        }else if(count%2==1){
          pose.squeezing_open_hand(targetAngles, totalTime, startTime);
        }
        updateCompleteFlag = false;
      }
    }
  }else if(currentTask==WAVING){
    M5.Lcd.setCursor(0,40);
    M5.Lcd.println(count);
    if(updateCompleteFlag){
      if(count==0){
        pose.waving_initial_arm(targetAngles, totalTime, startTime);
      }else if(count==total_count-1){
        pose.waving_straight(targetAngles, totalTime, startTime);
      }else if(count==total_count){
        currentTask=setNewTask(currentTask);
      }else if(count%2==1){
        pose.waving_left(targetAngles, totalTime, startTime);
      }else if(count%2==0){
        pose.waving_right(targetAngles, totalTime, startTime);
      }
      updateCompleteFlag = false;
      count++;
    }
  }else if(currentTask==BECKONING){
    M5.Lcd.setCursor(0,40);
    M5.Lcd.println(count);
    if(updateCompleteFlag){
      if(count==0){
        pose.beckoning_initial_arm(targetAngles, totalTime, startTime);
      }else if(count==total_count){
        currentTask=setNewTask(currentTask);
      }else if(count%2==1){
        pose.beckoning_inside(targetAngles, totalTime, startTime);
      }else if(count%2==0){
        pose.beckoning_outside(targetAngles, totalTime, startTime);
      }
      updateCompleteFlag = false;
      count++;
    }
  // }else if(currentTask==TAPPING){
  //   //send angle vector
  //   count--;
  //   if(count==0){
  //     currentTask=setNewTask(currentTask);
  //   }
  }  
}

std::string SoloMode::getName() {
  return "SoloMode";
}

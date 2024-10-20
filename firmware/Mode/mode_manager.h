#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <mode.h>
#include <solo_mode.h>
// #include <reflection_mode.h>
// #include <signaling_mode.h>
// #include <interaction_mode.h>

class ModeManager {
private:
    Mode* currentMode;
    SoloMode solo_mode;
    // ReflextionMode reflection_mode;
    // SignalingMode signaling_mode;
    // InteractionMode interaction_mode;

public:
    ModeManager() : currentMode(&solo_mode) {}

    void updateMode(uint8_t* force_data) {
        std::string nextModeName = currentMode->updateMode(force_data);
        // 現在のモード名と異なる場合、モードを変更
        if (nextModeName != currentMode->getName()) {
            if (nextModeName == "SoloMode") {
                currentMode = &solo_mode;
            // } else if (nextModeName == "ReflectionMode") {
            //     currentMode = &reflection_mode;
            // } else if (nextModeName == "SignalingMode") {
            //     currentMode = &signaling_mode;
            // } else if (nextModeName == "InteractionMode") {
            //     currentMode = &interaction_mode;
            }
        }
    }

    void processTask(int* targetAngles, float* totalTime, float* startTime, bool& updateCompleteFlag) {
        currentMode->processTask(targetAngles, totalTime, startTime, updateCompleteFlag);
    }
};


#endif 

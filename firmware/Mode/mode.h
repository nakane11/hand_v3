#ifndef MODE_H
#define MODE_H

#include <string>
#include <M5AtomS3.h>

class Mode {
public:
    virtual std::string updateMode(uint8_t* force_data) = 0;
    virtual void processTask(int* targetAngles, float* totalTime, float* startTime, bool& updateCompleteFlag) = 0;
    virtual std::string getName() = 0;
};

#endif

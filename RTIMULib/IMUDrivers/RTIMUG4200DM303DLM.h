////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#ifndef _RTIMUG4200DM303DLM_H
#define	_RTIMUG4200DM303DLM_H

#include "RTIMU.h"

//  Define this symbol to use cache mode

//#define G4200DM303DLM_CACHE_MODE   // not reliable at the moment

#ifdef G4200DM303DLM_CACHE_MODE

//  Cache defs

#define G4200DM303DLM_FIFO_CHUNK_SIZE    6                       // 6 bytes of gyro data
#define G4200DM303DLM_FIFO_THRESH        16                      // threshold point in fifo
#define G4200DM303DLM_CACHE_BLOCK_COUNT  16                      // number of cache blocks

typedef struct
{
    unsigned char data[G4200DM303DLM_FIFO_THRESH * G4200DM303DLM_FIFO_CHUNK_SIZE];
    int count;                                              // number of chunks in the cache block
    int index;                                              // current index into the cache
    unsigned char accel[6];                                 // the raw accel readings for the block
    unsigned char compass[6];                               // the raw compass readings for the block

} G4200DM303DLM_CACHE_BLOCK;

#endif

class RTIMUG4200DM303DLM : public RTIMU
{
public:
    RTIMUG4200DM303DLM(RTIMUSettings *settings);
    ~RTIMUG4200DM303DLM();

    virtual const char *IMUName() { return "L3G4200D + LSM303DLM"; }
    virtual int IMUType() { return RTIMU_TYPE_RTIMUG4200DM303DLM; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    bool setGyroSampleRate();
    bool setGyroCTRL2();
    bool setGyroCTRL4();
    bool setGyroCTRL5();
    bool setAccelCTRL1();
    bool setAccelCTRL2();
    bool setCompassCTRL5();
    bool setCompassCTRL6();
    bool setCompassCTRL7();

    unsigned char m_gyroSlaveAddr;                          // I2C address of L3GD20H
    unsigned char m_accelCompassSlaveAddr;                  // I2C address of LSM303D

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;

#ifdef G4200DM303DLM_CACHE_MODE
    bool m_firstTime;                                       // if first sample

    G4200DM303DLM_CACHE_BLOCK m_cache[G4200DM303DLM_CACHE_BLOCK_COUNT]; // the cache itself
    int m_cacheIn;                                          // the in index
    int m_cacheOut;                                         // the out index
    int m_cacheCount;                                       // number of used cache blocks

#endif
};

#endif // _RTIMUG4200DM303DLM_H

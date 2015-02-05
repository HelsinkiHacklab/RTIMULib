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


#include "RTIMUG4200DM303DLM.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

RTIMUG4200DM303DLM::RTIMUG4200DM303DLM(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMUG4200DM303DLM::~RTIMUG4200DM303DLM()
{
}

bool RTIMUG4200DM303DLM::IMUInit()
{
    unsigned char result;

#ifdef G4200DM303DLM_CACHE_MODE
    m_firstTime = true;
    m_cacheIn = m_cacheOut = m_cacheCount = 0;
#endif
    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    //  configure IMU

    m_gyroSlaveAddr = m_settings->m_I2CSlaveAddress;

    // work out accel/mag address

    if (m_settings->HALRead(LSM303DLM_ADDRESS0, LSM303DLM_WHO_AM_I, 1, &result, "")) {
        if (result == LSM303DLM_ID) {
            m_accelCompassSlaveAddr = LSM303DLM_ADDRESS0;
        }
    } else {
        m_accelCompassSlaveAddr = LSM303DLM_ADDRESS1;
    }

    setCalibrationData();

    //  enable the I2C bus

    if (!m_settings->HALOpen())
        return false;

    //  Set up the gyro

    if (!m_settings->HALWrite(m_gyroSlaveAddr, L3G4200D_CTRL5, 0x80, "Failed to boot L3G4200D"))
        return false;

    if (!m_settings->HALRead(m_gyroSlaveAddr, L3G4200D_WHO_AM_I, 1, &result, "Failed to read L3G4200D id"))
        return false;

    if (result != L3G4200D_ID) {
        HAL_ERROR1("Incorrect L3G4200D id %d\n", result);
        return false;
    }

    if (!setGyroSampleRate())
            return false;

    if (!setGyroCTRL2())
            return false;

    if (!setGyroCTRL4())
            return false;

    //  Set up the accel

    if (!setAccelCTRL1())
        return false;

    if (!setAccelCTRL4())
        return false;

    //  Set up the compass

    if (!setCompassCRA())
        return false;

    if (!setCompassCRB())
        return false;

    if (!setCompassCRM())
        return false;

#ifdef G4200DM303DLM_CACHE_MODE

    //  turn on gyro fifo

    if (!m_settings->HALWrite(m_gyroSlaveAddr, L3G4200D_FIFO_CTRL, 0x3f, "Failed to set L3G4200D FIFO mode"))
        return false;
#endif

    if (!setGyroCTRL5())
            return false;

    gyroBiasInit();

    HAL_INFO("G4200DM303DLM init complete\n");
    return true;
}

bool RTIMUG4200DM303DLM::setGyroSampleRate()
{
    unsigned char ctrl1;
    unsigned char lowOdr = 0;

    switch (m_settings->m_G4200DM303DLMGyroSampleRate) {
    case L3G4200D_SAMPLERATE_100:
        ctrl1 = 0x0f;
        m_sampleRate = 100;
        break;

    case L3G4200D_SAMPLERATE_200:
        ctrl1 = 0x4f;
        m_sampleRate = 200;
        break;

    case L3G4200D_SAMPLERATE_400:
        ctrl1 = 0x8f;
        m_sampleRate = 400;
        break;

    case L3G4200D_SAMPLERATE_800:
        ctrl1 = 0xcf;
        m_sampleRate = 800;
        break;

    default:
        HAL_ERROR1("Illegal L3G4200D sample rate code %d\n", m_settings->m_G4200DM303DLMGyroSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_G4200DM303DLMGyroBW) {
    case L3G4200D_BANDWIDTH_0:
        ctrl1 |= 0x00;
        break;

    case L3G4200D_BANDWIDTH_1:
        ctrl1 |= 0x10;
        break;

    case L3G4200D_BANDWIDTH_2:
        ctrl1 |= 0x20;
        break;

    case L3G4200D_BANDWIDTH_3:
        ctrl1 |= 0x30;
        break;

    }

    return (m_settings->HALWrite(m_gyroSlaveAddr, L3G4200D_CTRL1, ctrl1, "Failed to set L3G4200D CTRL1"));
}

bool RTIMUG4200DM303DLM::setGyroCTRL2()
{
    if ((m_settings->m_G4200DM303DLMGyroHpf < L3G4200D_HPF_0) || (m_settings->m_G4200DM303DLMGyroHpf > L3G4200D_HPF_9)) {
        HAL_ERROR1("Illegal L3G4200D high pass filter code %d\n", m_settings->m_G4200DM303DLMGyroHpf);
        return false;
    }
    return m_settings->HALWrite(m_gyroSlaveAddr,  L3G4200D_CTRL2, m_settings->m_G4200DM303DLMGyroHpf, "Failed to set L3G4200D CTRL2");
}

bool RTIMUG4200DM303DLM::setGyroCTRL4()
{
    unsigned char ctrl4;

    switch (m_settings->m_G4200DM303DLMGyroFsr) {
    case L3G4200D_FSR_250:
        ctrl4 = 0x00;
        m_gyroScale = (RTFLOAT)0.00875 * RTMATH_DEGREE_TO_RAD;
        break;

    case L3G4200D_FSR_500:
        ctrl4 = 0x10;
        m_gyroScale = (RTFLOAT)0.0175 * RTMATH_DEGREE_TO_RAD;
        break;

    case L3G4200D_FSR_2000:
        ctrl4 = 0x20;
        m_gyroScale = (RTFLOAT)0.07 * RTMATH_DEGREE_TO_RAD;
        break;

    default:
        HAL_ERROR1("Illegal L3G4200D FSR code %d\n", m_settings->m_G4200DM303DLMGyroFsr);
        return false;
    }

    return m_settings->HALWrite(m_gyroSlaveAddr,  L3G4200D_CTRL4, ctrl4, "Failed to set L3G4200D CTRL4");
}


bool RTIMUG4200DM303DLM::setGyroCTRL5()
{
    unsigned char ctrl5;

    //  Turn on hpf

    ctrl5 = 0x10;

#ifdef G4200DM303DLM_CACHE_MODE
    //  turn on fifo

    ctrl5 |= 0x40;
#endif

    return m_settings->HALWrite(m_gyroSlaveAddr,  L3G4200D_CTRL5, ctrl5, "Failed to set L3G4200D CTRL5");
}


bool RTIMUG4200DM303DLM::setAccelCTRL1()
{
    unsigned char ctrl1;

    if (
		m_settings->m_G4200DM303DLMAccelSampleRate != LSM303DLM_ACCEL_SAMPLERATE_0_5 &&
		m_settings->m_G4200DM303DLMAccelSampleRate != LSM303DLM_ACCEL_SAMPLERATE_1   &&
		m_settings->m_G4200DM303DLMAccelSampleRate != LSM303DLM_ACCEL_SAMPLERATE_2   &&
		m_settings->m_G4200DM303DLMAccelSampleRate != LSM303DLM_ACCEL_SAMPLERATE_5   &&
		m_settings->m_G4200DM303DLMAccelSampleRate != LSM303DLM_ACCEL_SAMPLERATE_10  &&
		m_settings->m_G4200DM303DLMAccelSampleRate != LSM303DLM_ACCEL_SAMPLERATE_37  &&
		m_settings->m_G4200DM303DLMAccelSampleRate != LSM303DLM_ACCEL_SAMPLERATE_74  &&
		m_settings->m_G4200DM303DLMAccelSampleRate != LSM303DLM_ACCEL_SAMPLERATE_292 &&
		m_settings->m_G4200DM303DLMAccelSampleRate != LSM303DLM_ACCEL_SAMPLERATE_780
	){
        HAL_ERROR1("Illegal LSM303DLM accel sample rate code %d\n", m_settings->m_G4200DM303DLMAccelSampleRate);
        return false;
    }

    ctrl1 = (m_settings->m_G4200DM303DLMAccelSampleRate << 3) | 0x07;

    return m_settings->HALWrite(m_accelCompassSlaveAddr,  LSM303DLM_CTRL1, ctrl1, "Failed to set LSM303DLM CTRL1");
}

bool RTIMUG4200DM303DLM::setAccelCTRL4()
{
    unsigned char ctrl4;

    switch (m_settings->m_G4200DM303DLMAccelFsr) {
    case LSM303DLM_ACCEL_FSR_2:
        m_accelScale = (RTFLOAT)0.000061;
        break;

    case LSM303DLM_ACCEL_FSR_4:
        m_accelScale = (RTFLOAT)0.000122;
        break;

    case LSM303DLM_ACCEL_FSR_8:
        m_accelScale = (RTFLOAT)0.000244;
        break;

    default:
        HAL_ERROR1("Illegal LSM303DLM accel FSR code %d\n", m_settings->m_G4200DM303DLMAccelFsr);
        return false;
    }

    ctrl4 = (m_settings->m_G4200DM303DLMAccelFsr << 4);

    return m_settings->HALWrite(m_accelCompassSlaveAddr,  LSM303DLM_CTRL2, ctrl4, "Failed to set LSM303DLM CTRL4");
}

bool RTIMUG4200DM303DLM::setCompassCRA()
{
    unsigned char cra;

    if ((m_settings->m_G4200DM303DLMCompassSampleRate < 0) || (m_settings->m_G4200DM303DLMCompassSampleRate > 7)) {
        HAL_ERROR1("Illegal LSM303DLM compass sample rate code %d\n", m_settings->m_G4200DM303DLMCompassSampleRate);
        return false;
    }

    cra = (m_settings->m_G4200DM303DLMCompassSampleRate << 2);

    return m_settings->HALWrite(m_compassSlaveAddr,  LSM303DLM_CRA_M, cra, "Failed to set LSM303DLM CRA_M");
}

bool RTIMUG4200DM303DLM::setCompassCRB()
{
    unsigned char crb;

    //  convert FSR to uT

    switch (m_settings->m_G4200DM303DLMCompassFsr) {
    case LSM303DLM_COMPASS_FSR_1_3:
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)1100;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)980;
        break;

    case LSM303DLM_COMPASS_FSR_1_9:
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)855;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)760;
       break;

    case LSM303DLM_COMPASS_FSR_2_5:
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)670;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)600;
        break;

    case LSM303DLM_COMPASS_FSR_4:
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)450;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)400;
        break;

    case LSM303DLM_COMPASS_FSR_4_7:
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)400;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)355;
        break;

    case LSM303DLM_COMPASS_FSR_5_6:
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)330;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)295;
        break;

    case LSM303DLM_COMPASS_FSR_8_1:
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)230;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)205;
        break;

    default:
        HAL_ERROR1("Illegal LSM303DLM compass FSR code %d\n", m_settings->m_G4200DM303DLMCompassFsr);
        return false;
    }

	crb = (m_settings->m_GD20M303DLMCompassFsr << 5);
    return m_settings->HALWrite(m_compassSlaveAddr,  LSM303DLM_CRB_M, crb, "Failed to set LSM303DLM CRB_M");
}

bool RTIMUG4200DM303DLM::setCompassCRM()
{
     return m_settings->HALWrite(m_compassSlaveAddr,  LSM303DLM_CRM_M, 0x00, "Failed to set LSM303DLM CRM_M");
}

int RTIMUG4200DM303DLM::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMUG4200DM303DLM::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];


#ifdef G4200DM303DLM_CACHE_MODE
    int count;

    if (!m_settings->HALRead(m_gyroSlaveAddr, L3G4200D_FIFO_SRC, 1, &status, "Failed to read L3G4200D fifo status"))
        return false;

    if ((status & 0x40) != 0) {
        HAL_INFO("L3G4200D fifo overrun\n");
        if (!m_settings->HALWrite(m_gyroSlaveAddr, L3G4200D_CTRL5, 0x10, "Failed to set L3G4200D CTRL5"))
            return false;

        if (!m_settings->HALWrite(m_gyroSlaveAddr, L3G4200D_FIFO_CTRL, 0x0, "Failed to set L3G4200D FIFO mode"))
            return false;

        if (!m_settings->HALWrite(m_gyroSlaveAddr, L3G4200D_FIFO_CTRL, 0x3f, "Failed to set L3G4200D FIFO mode"))
            return false;

        if (!setGyroCTRL5())
            return false;

        m_imuData.timestamp += m_sampleInterval * 32;
        return false;
    }

    // get count of samples in fifo
    count = status & 0x1f;

    if ((m_cacheCount == 0) && (count > 0) && (count < G4200DM303DLM_FIFO_THRESH)) {
        // special case of a small fifo and nothing cached - just handle as simple read

        if (!m_settings->HALRead(m_gyroSlaveAddr, 0x80 | L3G4200D_OUT_X_L, 6, gyroData, "Failed to read L3G4200D data"))
            return false;

        if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | LSM303DLM_OUT_X_L_A, 6, accelData, "Failed to read LSM303DLM accel data"))
            return false;

        if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | LSM303DLM_OUT_X_L_M, 6, compassData, "Failed to read LSM303DLM compass data"))
            return false;

        if (m_firstTime)
            m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
        else
            m_imuData.timestamp += m_sampleInterval;

        m_firstTime = false;
    } else {
        if (count >=  G4200DM303DLM_FIFO_THRESH) {
            // need to create a cache block

            if (m_cacheCount == G4200DM303DLM_CACHE_BLOCK_COUNT) {
                // all cache blocks are full - discard oldest and update timestamp to account for lost samples
                m_imuData.timestamp += m_sampleInterval * m_cache[m_cacheOut].count;
                if (++m_cacheOut == G4200DM303DLM_CACHE_BLOCK_COUNT)
                    m_cacheOut = 0;
                m_cacheCount--;
            }

            if (!m_settings->HALRead(m_gyroSlaveAddr, 0x80 | L3G4200D_OUT_X_L, G4200DM303DLM_FIFO_CHUNK_SIZE * G4200DM303DLM_FIFO_THRESH,
                         m_cache[m_cacheIn].data, "Failed to read L3G4200D fifo data"))
                return false;

            if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | LSM303DLM_OUT_X_L_A, 6,
                         m_cache[m_cacheIn].accel, "Failed to read LSM303DLM accel data"))
                return false;

            if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | LSM303DLM_OUT_X_L_M, 6,
                         m_cache[m_cacheIn].compass, "Failed to read LSM303DLM compass data"))
                return false;

            m_cache[m_cacheIn].count = G4200DM303DLM_FIFO_THRESH;
            m_cache[m_cacheIn].index = 0;

            m_cacheCount++;
            if (++m_cacheIn == G4200DM303DLM_CACHE_BLOCK_COUNT)
                m_cacheIn = 0;

        }

        //  now fifo has been read if necessary, get something to process

        if (m_cacheCount == 0)
            return false;

        memcpy(gyroData, m_cache[m_cacheOut].data + m_cache[m_cacheOut].index, G4200DM303DLM_FIFO_CHUNK_SIZE);
        memcpy(accelData, m_cache[m_cacheOut].accel, 6);
        memcpy(compassData, m_cache[m_cacheOut].compass, 6);

        m_cache[m_cacheOut].index += G4200DM303DLM_FIFO_CHUNK_SIZE;

        if (--m_cache[m_cacheOut].count == 0) {
            //  this cache block is now empty

            if (++m_cacheOut == G4200DM303DLM_CACHE_BLOCK_COUNT)
                m_cacheOut = 0;
            m_cacheCount--;
        }
        if (m_firstTime)
            m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
        else
            m_imuData.timestamp += m_sampleInterval;

        m_firstTime = false;
    }

#else
    if (!m_settings->HALRead(m_gyroSlaveAddr, L3G4200D_STATUS, 1, &status, "Failed to read L3G4200D status"))
        return false;

    if ((status & 0x8) == 0)
        return false;

    if (!m_settings->HALRead(m_gyroSlaveAddr, 0x80 | L3G4200D_OUT_X_L, 6, gyroData, "Failed to read L3G4200D data"))
        return false;

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | LSM303DLM_OUT_X_L_A, 6, accelData, "Failed to read LSM303DLM accel data"))
        return false;

    if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | LSM303DLM_OUT_X_L_M, 6, compassData, "Failed to read LSM303DLM compass data"))
        return false;

#endif

    RTMath::convertToVector(gyroData, m_imuData.gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_imuData.accel, m_accelScale, false);
    RTMath::convertToVector(compassData, m_imuData.compass, m_compassScale, false);

    //  sort out gyro axes

    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());

    //  sort out compass axes

    m_imuData.compass.setY(-m_imuData.compass.y());
    m_imuData.compass.setZ(-m_imuData.compass.z());

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    //  now update the filter

    updateFusion();

    return true;
}

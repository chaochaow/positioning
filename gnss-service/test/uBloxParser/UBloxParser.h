/**************************************************************************
* @licence app begin@
*
* SPDX-License-Identifier: MPL-2.0
*
* \ingroup GNSSService
* \author Chao Wang <chao.wang@garmin.com>
*
* \copyright Copyright (C) 2016, Garmin International
* 
* \license
* This Source Code Form is subject to the terms of the
* Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with
* this file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* @licence end@
**************************************************************************/


#ifndef UBLOXPARSER_H
#define UBLOXPARSER_H

#include <memory>
#include <vector>
#include "gnss.h"

#define UBLOX_MSG_LENGTH (512)

typedef enum 
{
    UBLOX_CLASS_NAV             = 0x00000001,
    UBLOX_CLASS_RXM             = 0x00000002,
    UBLOX_CLASS_INF             = 0x00000004,
    UBLOX_CLASS_ACK             = 0x00000005,
    UBLOX_CLASS_CFG             = 0x00000006,
    UBLOX_CLASS_MGA             = 0x00000013
}UBloxClassID;

typedef enum 
{
    UBLOX_NAV_PVT               = 0x00000007,
    UBLOX_NAV_SAT               = 0x00000035
}UBloxNavID;

typedef enum
{
    UBLOX_MSG_UNKWN,
    UBLOX_MSG_SYNC1,
    UBLOX_MSG_SYNC2,
    UBLOX_MSG_CLASS,
    UBLOX_MSG_ID,
    UBLOX_MSG_LEN1,
    UBLOX_MSG_LEN2,
    UBLOX_MSG_BODY,
    UBLOX_MSG_CHSUM
}UBloxMsgStatus;

typedef enum
{
    UBLOX_GNSS_GPS,
    UBLOX_GNSS_SBAS,
    UBLOX_GNSS_GALILEO,
    UBLOX_GNSS_BEIDOU,
    UBLOX_GNSS_IMES,
    UBLOX_GNSS_QZSS,
    UBLOX_GNSS_GLONASS
}UBloxGnssType;


typedef enum
{
    //UBLOX_SAT_QUALITYIND  = 0x00000004,
    UBLOX_SAT_SATUSED       = 0x00000008,
    UBLOX_SAT_HEALTH        = 0x00000020,
    UBLOX_SAT_DIFFCORR      = 0x00000040,
    UBLOX_SAT_SMOOTHED      = 0x00000080,
    //UBLOX_SAT_ORBITSOURCE = 0x00000400,
    UBLOX_SAT_EPHEMAVAIL    = 0x00000800
    //UBLOX_SAT_ALMAVAIL    = 0x00001000,
}UBloxSatFlag;

typedef struct
{
    uint8_t classId;
    uint8_t msgId;
    uint16_t msgLen;
    uint8_t msgBody[UBLOX_MSG_LENGTH];
    uint8_t checkSumA;
    uint8_t checkSumB;
    uint16_t msgDataIndex;

    void Reset()
    {
        classId = 0;
        msgId = 0;
        msgLen = 0;
        std::memset(msgBody, 0, sizeof(uint8_t) * UBLOX_MSG_LENGTH);
        checkSumA = 0;
        checkSumB = 0;
        msgDataIndex = 0;
    }
}UBloxMsg;

typedef struct
{
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t reserved1[6];
    uint32_t headVeh;
    uint8_t reserved2[4];

}UBloxPVTStruct;

typedef struct
{
    uint32_t iTOW;
    uint8_t version;
    uint8_t numSvs;
    uint8_t reserved1[2];
}UBloxSatStruct;

typedef struct
{
    uint8_t gnssId;
    uint8_t svId;
    uint8_t cn0;
    int8_t elev;
    int16_t azim;
    int16_t prRes;
    uint32_t flags;
}UBloxSatBlock;

typedef enum
{
    UBLOX_PVT_DATA_READY            = 0x00000001,
    UBLOX_SAT_DATA_READY            = 0x00000002,
    UBLOX_DATA_READY_FOR_OUTPUT     = (UBLOX_PVT_DATA_READY | UBLOX_SAT_DATA_READY)
};

class UBloxParser
{
public:
    bool ProcessDataInput(uint8_t ch);
    UBloxParser();
    ~UBloxParser();
private:

    UBloxMsgStatus msgStatus;
    UBloxMsg msg;
    uint32_t dataAvailMask;
    int32_t lastPvtDataTow; //in milli-seconds
    int32_t lastSatDataTow; //in milli-seconds

    TGNSSPosition gnssData;
    std::vector<TGNSSSatelliteDetail> gnssSatInfo;

    void CalculateCheckSum(uint8_t * dataBuf, uint16_t msgLen, uint8_t & checkSumA, uint8_t & checkSumB);
    bool FrameMsg(uint8_t ch);
    bool DecodeNavSat(uint8_t * buf);
    bool DecodeNavPVT(uint8_t * buf);
    void Reset();
};

#endif
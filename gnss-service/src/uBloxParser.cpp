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
#include <math.h>
#include "uBloxParser.h"

UBloxParser::UBloxParser()
{
    Reset();
}

UBloxParser::~UBloxParser()
{

}

bool UBloxParser::ProcessDataInput(unsigned char ch)
{

    if(true == FrameMsg(ch))
    {
        if(UBLOX_CLASS_NAV == msg.classId)
        {
            if( UBLOX_NAV_PVT == msg.msgId )
            {
                dataAvailType= UBLOX_PVT_DATA_READY;
                return true;
            }
            else if( UBLOX_NAV_SAT == msg.msgId )
            {
                dataAvailType= UBLOX_SAT_DATA_READY;
                return true;
            }

        }

    }
    return false;
}

bool UBloxParser::FrameMsg(unsigned char ch)
{
    switch (msgStatus)
    {
    case UBLOX_MSG_UNKWN:
        if(0xb5 == ch)
        {
            msgStatus = UBLOX_MSG_SYNC2;
        }
        break;

    case UBLOX_MSG_SYNC2:
        if( 0x62 == ch)
        {
            msgStatus = UBLOX_MSG_CLASS;
        }
        else
        {
            msgStatus = UBLOX_MSG_UNKWN;
        }
        break;

    case UBLOX_MSG_CLASS:
        msg.Reset();
        msg.classId = ch;
        msgStatus = UBLOX_MSG_ID;
        break;

    case UBLOX_MSG_ID:
        msg.msgId = ch;
        msgStatus = UBLOX_MSG_LEN1;
        break;

    case UBLOX_MSG_LEN1:
        msg.msgLen = ch;
        msgStatus = UBLOX_MSG_LEN2;
        break;

    case UBLOX_MSG_LEN2:
        msg.msgLen += ch<<8;
        msgStatus = UBLOX_MSG_BODY;
        break;

    case UBLOX_MSG_BODY:
        if(msg.msgDataIndex < msg.msgLen)
        {
            msg.msgBody[msg.msgDataIndex++] = ch;
        }
        else
        {
            msg.checkSumA = ch;
            msgStatus = UBLOX_MSG_CHSUM;
        }
        break;

    case UBLOX_MSG_CHSUM:
        msg.checkSumB = ch;
        uint8_t chckSumA, chckSumB;

        //add  classId, msgId and msgLen into the checksum calculation
        CalculateCheckSum((uint8_t *)&msg, msg.msgLen+4, chckSumA, chckSumB);

        if((chckSumA == msg.checkSumA) && (chckSumB = msg.checkSumB))
        {
            msgStatus = UBLOX_MSG_UNKWN;
            return true;
        }
        else
        {
            msgStatus = UBLOX_MSG_UNKWN;
        }

    default:
        break;
    }
    return false;

}
void UBloxParser::CalculateCheckSum(uint8_t * dataBuf, uint16_t msgLen, uint8_t & checkSumA, uint8_t & checkSumB)
{
    checkSumA = checkSumB = 0;

    for(uint16_t i = 0; i < msgLen; i++)
    {
        checkSumA += dataBuf[i];
        checkSumB += checkSumA;
    }
}

void UBloxParser::Reset()
{
    msgStatus = UBLOX_MSG_UNKWN;
    dataAvailType = UBLOX_DATA_UNKNOWN;
    msg.Reset();
}

bool UBloxParser::GetGnssPvtData(UBloxPVTStruct ** gnss)
{
    if(UBLOX_PVT_DATA_READY == dataAvailType)
    {
        *gnss = (UBloxPVTStruct *) msg.msgBody;
        dataAvailType = UBLOX_DATA_UNKNOWN;
        return true;
    }
    return false;
}

bool UBloxParser::GetGnssSatdata(UBloxSatStruct ** satHeader, UBloxSatBlock ** satBlock)
{
    if(UBLOX_SAT_DATA_READY == dataAvailType)
    {
        *satHeader = (UBloxSatStruct *) msg.msgBody;

        *satBlock = (UBloxSatBlock *)&msg.msgBody[8];
        dataAvailType = UBLOX_DATA_UNKNOWN;
        return true;
    }
    return false;
}

bool UBloxParser::GetGnssMeasData(UBloxMeasHeader ** measHeader, UBloxMeasBlock ** measBlock)
{
    if(UBLOX_MEAS_DATA_READY == dataAvailType)
    {
        *measHeader = (UBloxMeasHeader *) msg.msgBody;

        *measBlock = (UBloxMeasBlock *)&msg.msgBody[16];
        dataAvailType = UBLOX_DATA_UNKNOWN;
        return true;
    }
    return false;
}

bool UBloxParser::GetGpsEphData(UBloxGpsEphStruct ** eph)
{
    if(UBLOX_EPH_DATA_READY == dataAvailType)
    {
        *eph = (UBloxGpsEphStruct *) msg.msgBody;
        dataAvailType = UBLOX_DATA_UNKNOWN;
        return true;
    }
    return false;
}

bool UBloxParser::GetGpsUtcData(UBloxGpsUtcStruct ** utc)
{
    if(UBLOX_UTC_DATA_READY == dataAvailType)
    {
        *utc = (UBloxGpsUtcStruct *) msg.msgBody;
        dataAvailType = UBLOX_DATA_UNKNOWN;
        return true;
    }
    return false;
}

bool UBloxParser::GetGpsIonoData(UBloxGpsIonoStruct ** iono)
{
    if(UBLOX_IONO_DATA_READY == dataAvailType)
    {
        *iono = (UBloxGpsIonoStruct *) msg.msgBody;
        dataAvailType = UBLOX_DATA_UNKNOWN;
        return true;
    }
    return false;
}
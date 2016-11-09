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

#include "UBloxParser.h"

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
            switch (msg.msgId)
            {
            case UBLOX_NAV_PVT:
                DecodeNavPVT(msg.msgBody);
                break;

            case UBLOX_NAV_SAT:
                DecodeNavSat(msg.msgBody);
                break;

            default:
                break;
            }
        }

        if(UBLOX_DATA_READY_FOR_OUTPUT == dataAvailMask)
        {
            //send the data to the application
        }
    }
    return true;
}

bool UBloxParser::FrameMsg(unsigned char ch)
{
    switch (msgStatus)
    {
    case UBLOX_MSG_UNKWN:
        if(0xb5 == ch)
        {
            msgStatus = UBLOX_MSG_SYNC1;
        }
        break;

    case UBLOX_MSG_SYNC1:
        if( 0x62 == ch)
        {
            msgStatus = UBLOX_MSG_SYNC2;
        }
        else
        {
            msgStatus = UBLOX_MSG_UNKWN;
        }
        break;

    case UBLOX_MSG_SYNC2:
        if( UBLOX_MSG_SYNC2 == ch)
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

    default:
        break;
    }
    return false;

}

bool UBloxParser::DecodeNavSat(uint8_t * buf)
{

    UBloxSatStruct * pSatHeader = (UBloxSatStruct *) buf;

    if(pSatHeader->iTOW != lastPvtDataTow)
    {
        dataAvailMask = 0;
    }
    lastSatDataTow = pSatHeader->iTOW;
    dataAvailMask |= UBLOX_SAT_DATA_READY;

    gnssData.visibleSatellites = pSatHeader->numSvs;
    gnssData.validityBits |= GNSS_POSITION_VSAT_VALID;

    uint8_t numTracked = 0;
    uint32_t gnssConstMask = 0;

    gnssSatInfo.clear();


    for(uint8_t i = 0; i < gnssData.visibleSatellites; i++)
    {
        //8 byte of the common sat info
        //12 bytes for each satellite block
        UBloxSatBlock * pBlock = (UBloxSatBlock *) &buf[8+i*12];
        TGNSSSatelliteDetail sat;

        memset(&sat, 0, sizeof(sat));

        sat.elevation = pBlock->elev;
        sat.validityBits |= GNSS_SATELLITE_ELEVATION_VALID;

        sat.azimuth = pBlock->azim;
        sat.validityBits |= GNSS_SATELLITE_AZIMUTH_VALID;

        sat.CNo = pBlock->cn0;
        sat.validityBits |= GNSS_SATELLITE_CNO_VALID;

        if(sat.CNo > 0.0)
        {
            numTracked++;
        }

        //assign TOW to timestamp right now
        //need to find another way to fill this field
        sat.timestamp = pSatHeader->iTOW;

        sat.satelliteId = pBlock->svId;
        sat.validityBits |= GNSS_SATELLITE_ID_VALID;

        sat.posResidual = static_cast<int16_t>(pBlock->prRes * 0.1);
        sat.validityBits |= GNSS_SATELLITE_RESIDUAL_VALID;

        sat.validityBits |= GNSS_SATELLITE_SYSTEM_VALID;
        switch (pBlock->gnssId)
        {
        case UBLOX_GNSS_GPS:
            sat.system = GNSS_SYSTEM_GPS;
            gnssConstMask |= GNSS_SYSTEM_GPS;
            break;
        case UBLOX_GNSS_SBAS:
            sat.system = GNSS_SYSTEM_SBAS_WAAS;
            gnssConstMask |= GNSS_SYSTEM_SBAS_WAAS;
            break;
        case UBLOX_GNSS_GALILEO:
            sat.system = GNSS_SYSTEM_GALILEO;
            gnssConstMask |= GNSS_SYSTEM_GALILEO;
            break;
        case UBLOX_GNSS_BEIDOU:
            sat.system = GNSS_SYSTEM_BEIDOU;
            gnssConstMask |= GNSS_SYSTEM_BEIDOU;
            break;
        case UBLOX_GNSS_IMES:
            sat.validityBits &= ~GNSS_SATELLITE_SYSTEM_VALID;
            break;
        case UBLOX_GNSS_QZSS:
            sat.system = GNSS_SYSTEM_SBAS_QZSS_SAIF;
            gnssConstMask |= GNSS_SYSTEM_SBAS_QZSS_SAIF;
            break;
        case UBLOX_GNSS_GLONASS:
            sat.system = GNSS_SYSTEM_GLONASS;
            gnssConstMask |= GNSS_SYSTEM_GLONASS;
            break;
        default:
            sat.validityBits &= ~GNSS_SATELLITE_SYSTEM_VALID;
            break;
        }

        if(pBlock->flags & UBLOX_SAT_SATUSED)
        {
            sat.statusBits |= GNSS_SATELLITE_USED;
        }
        sat.validityBits |= GNSS_SATELLITE_USED_VALID;

        if(pBlock->flags & UBLOX_SAT_EPHEMAVAIL)
        {
            sat.statusBits |= GNSS_SATELLITE_EPHEMERIS_AVAILABLE;
        }
        sat.validityBits |= GNSS_SATELLITE_EPHEMERIS_AVAILABLE_VALID;

        gnssSatInfo.push_back(sat);
    }

    gnssData.activatedSystems = gnssConstMask;
    gnssData.validityBits |= GNSS_POSITION_USYS_VALID;

    gnssData.trackedSatellites = numTracked;
    gnssData.validityBits |= GNSS_POSITION_TSAT_VALID;

    return true;
}

bool UBloxParser::DecodeNavPVT(uint8_t * buf)
{
    UBloxPVTStruct * pPVT = (UBloxPVTStruct *) buf;

    if(pPVT->iTOW != lastSatDataTow)
    {
        dataAvailMask = 0;
    }

    lastPvtDataTow = pPVT->iTOW;
    dataAvailMask |= UBLOX_PVT_DATA_READY;

    //this needs to be updated, check the definition of timestamp
    gnssData.timestamp = pPVT->iTOW;

    gnssData.longitude = pPVT->lon * 1e-7;
    gnssData.validityBits |= GNSS_POSITION_LONGITUDE_VALID;

    gnssData.latitude = pPVT->lat * 1e-7;
    gnssData.validityBits |= GNSS_POSITION_LATITUDE_VALID;

    gnssData.altitudeEll = static_cast<float>(pPVT->height * 0.001);
    gnssData.validityBits |= GNSS_POSITION_ALTITUDEELL_VALID;

    gnssData.altitudeMSL = static_cast<float>(pPVT->hMSL * 0.001);
    gnssData.validityBits |= GNSS_POSITION_ALTITUDEMSL_VALID;

    gnssData.hSpeed = static_cast<float>(sqrt(pPVT->velN * pPVT->velN + pPVT->velE * pPVT->velE) * 0.001);
    gnssData.validityBits |= GNSS_POSITION_HSPEED_VALID;

    gnssData.vSpeed =static_cast<float>(- pPVT->velD * 0.001);
    gnssData.validityBits |= GNSS_POSITION_VSPEED_VALID;

    gnssData.heading = static_cast<float>(pPVT->headMot * 1e-5);
    gnssData.validityBits |= GNSS_POSITION_HEADING_VALID;

    gnssData.pdop = static_cast<float>(pPVT->pDOP * 0.01);
    gnssData.validityBits |= GNSS_POSITION_PDOP_VALID;

    gnssData.usedSatellites = pPVT->numSV;
    gnssData.validityBits |= GNSS_POSITION_USAT_VALID;

    //uBlox does not provide HDOP in PVT message, take half of the PDOP as HDOP
    //gnssData.hdop = static_cast<float>(0.717 * gnssData.pdop);
    //gnssData.vdop = gnssData.hdop;
    gnssData.sigmaHPosition = static_cast<float>(pPVT->hAcc * 0.001);
    gnssData.validityBits |= GNSS_POSITION_SHPOS_VALID;

    gnssData.sigmaAltitude = static_cast<float>(pPVT->vAcc * 0.001);
    gnssData.validityBits |= GNSS_POSITION_SALT_VALID;

    gnssData.sigmaHeading = static_cast<float>(pPVT->headAcc * 1e-5);
    gnssData.validityBits |= GNSS_POSITION_SHEADING_VALID;

    gnssData.sigmaHSpeed = static_cast<float>(0.717 * pPVT->sAcc * 0.001);
    gnssData.validityBits |= GNSS_POSITION_SHSPEED_VALID;

    gnssData.sigmaVSpeed = gnssData.sigmaHSpeed;
    gnssData.validityBits |= GNSS_POSITION_SVSPEED_VALID;


    switch (pPVT->fixType)
    {
    case 0:
    case 1:
        gnssData.fixStatus = GNSS_FIX_STATUS_NO;
        gnssData.validityBits |= GNSS_POSITION_STAT_VALID;
        break;
    case 2:
        gnssData.fixStatus = GNSS_FIX_STATUS_2D;
        gnssData.validityBits |= GNSS_POSITION_STAT_VALID;
        break;
    case 3:
        gnssData.fixStatus = GNSS_FIX_STATUS_3D;
        gnssData.validityBits |= GNSS_POSITION_STAT_VALID;
        break;
    default:
        break;

    }


    return true;

}

void UBloxParser::CalculateCheckSum(uint8_t * dataBuf, uint16_t msgLen, uint8_t & checkSumA, uint8_t & checkSumB)
{
    checkSumA = checkSumB = 0;

    for(uint8_t i = 0; i < msgLen; i++)
    {
        checkSumA += dataBuf[i];
        checkSumB += checkSumA;
    }
}

void UBloxParser::Reset()
{
    msgStatus = UBLOX_MSG_UNKWN;
    dataAvailMask = 0;
    msg.Reset();
    lastSatDataTow = lastPvtDataTow = -1;
}
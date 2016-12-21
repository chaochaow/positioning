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
#include <stdio.h>
#include "../api/gnss.h"
//#include "globals.h"
#include "uBloxParser.h"

static const uint16_t MAX_GNSS_SAT_CHANNEL = 26;

UBloxParser parser;

void PrintGnssSol(TGNSSPosition position);
void PrintGnssInfo(TGNSSSatelliteDetail  * satInfo, uint16_t satNum);
bool ExtractGnssPvtData(TGNSSPosition* gnssData);
bool ExtractSatelliteDetails(TGNSSSatelliteDetail* satelliteDetails, TGNSSPosition* gnssData);

int main(int argc, char * argv[])
{

    if(argc < 2)
    {
        printf("Please provide file input\n");
        return 0;
    }

    FILE * f_in = fopen(argv[1], "rb");
    uint8_t ch;

    uint32_t dataAvailMask = 0;
    uint64_t lastSatDataTow = 604800001;
    uint64_t lastPvtDataTow = 604800001;

    TGNSSPosition pvtData;
    TGNSSSatelliteDetail satInfo[MAX_GNSS_SAT_CHANNEL];
    uint16_t numSat;

    while(!feof(f_in))
    {
        ch = fgetc(f_in);
        if(true == parser.ProcessDataInput(ch))
        {
            uint32_t dataType = parser.GetUBloxDataType() ;

            if(UBLOX_PVT_DATA_READY == dataType)
            {
                if(true == ExtractGnssPvtData(&pvtData))
                {
                    if(pvtData.timestamp != lastSatDataTow)
                    {
                        dataAvailMask = 0;
                    }
                    dataAvailMask |= UBLOX_PVT_DATA_READY;
                    lastPvtDataTow = pvtData.timestamp;
                }
            }
            else if(UBLOX_SAT_DATA_READY == dataType)
            {
                if(true == ExtractSatelliteDetails(satInfo, &pvtData))
                {
                    if(satInfo[0].timestamp !=  lastPvtDataTow)
                    {
                        dataAvailMask = 0;
                    }
                    dataAvailMask |= UBLOX_SAT_DATA_READY;
                    lastSatDataTow = satInfo[0].timestamp;
                }
            }

            if(UBLOX_DATA_READY_FOR_OUTPUT == dataAvailMask)
            {
                //updateGNSSPosition(pvtData, 1);
                //updateGNSSSatelliteDetail(satInfo, pvtData.visibleSatellites);
                PrintGnssSol(pvtData);
                PrintGnssInfo(satInfo, pvtData.visibleSatellites);
            }
        }
    }
    return 1;
}

bool ExtractGnssPvtData(TGNSSPosition* gnssData)
{
    UBloxPVTStruct * pPVT = NULL;
    if(true == parser.GetGnssPvtData(&pPVT))
    {
        //this needs to be updated, check the definition of timestamp
        gnssData->timestamp = pPVT->iTOW;

        gnssData->longitude = pPVT->lon * 1e-7;
        gnssData->validityBits |= GNSS_POSITION_LONGITUDE_VALID;

        gnssData->latitude = pPVT->lat * 1e-7;
        gnssData->validityBits |= GNSS_POSITION_LATITUDE_VALID;

        gnssData->altitudeEll = static_cast<float>(pPVT->height * 0.001);
        gnssData->validityBits |= GNSS_POSITION_ALTITUDEELL_VALID;

        gnssData->altitudeMSL = static_cast<float>(pPVT->hMSL * 0.001);
        gnssData->validityBits |= GNSS_POSITION_ALTITUDEMSL_VALID;

        gnssData->hSpeed = static_cast<float>(sqrt(pPVT->velN * pPVT->velN + pPVT->velE * pPVT->velE) * 0.001);
        gnssData->validityBits |= GNSS_POSITION_HSPEED_VALID;

        gnssData->vSpeed =static_cast<float>(- pPVT->velD * 0.001);
        gnssData->validityBits |= GNSS_POSITION_VSPEED_VALID;

        gnssData->heading = static_cast<float>(pPVT->headMot * 1e-5);
        gnssData->validityBits |= GNSS_POSITION_HEADING_VALID;

        gnssData->pdop = static_cast<float>(pPVT->pDOP * 0.01);
        gnssData->validityBits |= GNSS_POSITION_PDOP_VALID;

        gnssData->usedSatellites = pPVT->numSV;
        gnssData->validityBits |= GNSS_POSITION_USAT_VALID;

        //uBlox does not provide HDOP in PVT message, take half of the PDOP as HDOP
        //gnssData->hdop = static_cast<float>(0.717 * gnssData->pdop);
        //gnssData->vdop = gnssData->hdop;
        gnssData->sigmaHPosition = static_cast<float>(pPVT->hAcc * 0.001);
        gnssData->validityBits |= GNSS_POSITION_SHPOS_VALID;

        gnssData->sigmaAltitude = static_cast<float>(pPVT->vAcc * 0.001);
        gnssData->validityBits |= GNSS_POSITION_SALT_VALID;

        gnssData->sigmaHeading = static_cast<float>(pPVT->headAcc * 1e-5);
        gnssData->validityBits |= GNSS_POSITION_SHEADING_VALID;

        gnssData->sigmaHSpeed = static_cast<float>(0.717 * pPVT->sAcc * 0.001);
        gnssData->validityBits |= GNSS_POSITION_SHSPEED_VALID;

        gnssData->sigmaVSpeed = gnssData->sigmaHSpeed;
        gnssData->validityBits |= GNSS_POSITION_SVSPEED_VALID;


        switch (pPVT->fixType)
        {
        case 0:
        case 1:
            gnssData->fixStatus = GNSS_FIX_STATUS_NO;
            gnssData->validityBits |= GNSS_POSITION_STAT_VALID;
            break;
        case 2:
            gnssData->fixStatus = GNSS_FIX_STATUS_2D;
            gnssData->validityBits |= GNSS_POSITION_STAT_VALID;
            break;
        case 3:
            gnssData->fixStatus = GNSS_FIX_STATUS_3D;
            gnssData->validityBits |= GNSS_POSITION_STAT_VALID;
            break;
        default:
            break;

        }

        if(gnssData->fixStatus > GNSS_FIX_STATUS_NO)
        {
            gnssData->fixTypeBits |= GNSS_FIX_TYPE_SINGLE_FREQUENCY;
            gnssData->fixTypeBits |= GNSS_FIX_TYPE_ESTIMATED;
            gnssData->validityBits |= GNSS_POSITION_TYPE_VALID;
        }
        return true;

    }
    else
    {
        return false;
    }
}

bool ExtractSatelliteDetails(TGNSSSatelliteDetail* satelliteDetails, TGNSSPosition* gnssData)
{
    UBloxSatStruct * pSatHeader = NULL;
    UBloxSatBlock * pSatBlocks = NULL;

    if(true == parser.GetGnssSatdata(&pSatHeader, &pSatBlocks))
    {
        gnssData->visibleSatellites = pSatHeader->numSvs;
        if(pSatHeader->numSvs > MAX_GNSS_SAT_CHANNEL)
        {
            gnssData->visibleSatellites = MAX_GNSS_SAT_CHANNEL;
        }
        gnssData->validityBits |= GNSS_POSITION_VSAT_VALID;

        uint8_t numTracked = 0;
        uint32_t gnssConstMask = 0;

        for(uint8_t i = 0; i < gnssData->visibleSatellites; i++)
        {
            //8 byte of the common sat info
            //12 bytes for each satellite block
            UBloxSatBlock * pBlock = &pSatBlocks[i];
            TGNSSSatelliteDetail & sat = satelliteDetails[i];

            memset(&sat, 0, sizeof(sat));

            if(pBlock->elev >= 0 && pBlock->elev < 91)
            {
                sat.elevation = pBlock->elev;
                sat.validityBits |= GNSS_SATELLITE_ELEVATION_VALID;
            }

            if(pBlock->azim >=0 && pBlock->azim < 361)
            {
                sat.azimuth = pBlock->azim;
                sat.validityBits |= GNSS_SATELLITE_AZIMUTH_VALID;
            }

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

            //when sat is not used in nav, residual is zero
            sat.posResidual = static_cast<int16_t>(pBlock->prRes * 0.1);

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
                sat.satelliteId -= 87;
                break;
            case UBLOX_GNSS_GALILEO:
                sat.system = GNSS_SYSTEM_GALILEO;
                gnssConstMask |= GNSS_SYSTEM_GALILEO;
                sat.satelliteId -= 210;
                break;
            case UBLOX_GNSS_BEIDOU:
                sat.system = GNSS_SYSTEM_BEIDOU;
                gnssConstMask |= GNSS_SYSTEM_BEIDOU;
                //sat id needs to be further defined
                break;
            case UBLOX_GNSS_IMES:
                //sat id needs to be further defined
                break;
            case UBLOX_GNSS_QZSS:
                sat.system = GNSS_SYSTEM_SBAS_QZSS_SAIF;
                gnssConstMask |= GNSS_SYSTEM_SBAS_QZSS_SAIF;
                //sat id needs to be further defined
                break;
            case UBLOX_GNSS_GLONASS:
                sat.system = GNSS_SYSTEM_GLONASS;
                gnssConstMask |= GNSS_SYSTEM_GLONASS;
                sat.satelliteId += 64;
                break;
            default:
                sat.validityBits &= ~GNSS_SATELLITE_SYSTEM_VALID;
                break;
            }

            if(pBlock->flags & UBLOX_SAT_SATUSED)
            {
                sat.statusBits |= GNSS_SATELLITE_USED;
                sat.validityBits |= GNSS_SATELLITE_RESIDUAL_VALID;
                sat.validityBits |= GNSS_SATELLITE_USED_VALID;
            }

            if(pBlock->flags & UBLOX_SAT_EPHEMAVAIL)
            {
                sat.statusBits |= GNSS_SATELLITE_EPHEMERIS_AVAILABLE;
                sat.validityBits |= GNSS_SATELLITE_EPHEMERIS_AVAILABLE_VALID;
            }

        }

        gnssData->usedSystems = gnssConstMask;
        gnssData->validityBits |= GNSS_POSITION_USYS_VALID;

        gnssData->trackedSatellites = numTracked;
        gnssData->validityBits |= GNSS_POSITION_TSAT_VALID;

        return true;

    }
    else
    {
        return false;
    }
}

void PrintGnssSol(TGNSSPosition position)
{
    static FILE * f_pvt = NULL;
    if(NULL == f_pvt)
    {
        f_pvt = fopen("sol.txt","wb");
        fprintf(f_pvt, "%%time  lat     lon     height      altitude    hSpeed      vSpeed      heading    pdop    hdop    vdop    usedSat     trackedSat      visibleSat      sigPos      sigAlt  sigHSpeed   sigHeading  fixedStatus  validitybits\n");
    }

    fprintf(f_pvt, "%12.3lf %12.8lf %12.8lf %10.4lf %10.4lf %8.4lf %8.4lf %8.4lf %5.2lf %5.2lf %5.2lf %2d %2d %2d %8.4lf %8.4lf %8.4lf %8.4lf %d %08x\n",
        position.timestamp * 0.001, position.latitude, position.longitude, position.altitudeEll, position.altitudeMSL, position.hSpeed, position.vSpeed, position.heading, position.pdop, position.hdop,
        position.vdop, position.usedSatellites, position.trackedSatellites, position.visibleSatellites, position.sigmaHPosition, position.sigmaAltitude, position.sigmaHSpeed, position.sigmaHeading, position.fixStatus, position.validityBits);
    fflush(f_pvt);

}

void PrintGnssInfo(TGNSSSatelliteDetail  * satInfo, uint16_t satNum)
{
    static FILE * f_sat = NULL;
    if(NULL == f_sat)
    {
        f_sat = fopen("sat.txt","wb");
        fprintf(f_sat, "%%time  system     id     azimuth      elevation    CN0      statusBits      residuals    validitybits\n");
    }

    for(uint16_t count = 0; count < satNum; count++)
    {
        TGNSSSatelliteDetail & sat = satInfo[count];
        fprintf(f_sat, "%12.3lf %10d %3d %3d %2d %2d %02x %4d %02x\n",
            sat.timestamp * 0.001, sat.system, sat.satelliteId, sat.azimuth, sat.elevation, sat.CNo, sat.statusBits, sat.posResidual, sat.validityBits);
    }
    fflush(f_sat);
}


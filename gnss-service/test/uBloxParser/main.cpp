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

UBloxParser parser;
void PrintGnssSol(TGNSSPosition position);
void PrintGnssInfo(TGNSSSatelliteDetail  * satInfo, uint16_t satNum);



void main(int argc, char * argv[])
{

    if(argc < 2)
    {
        printf("Please provide file input\n");
        return;
    }

    FILE * f_in = fopen(argv[1], "rb");
    uint8_t ch;

    parser.Reset();
    while(!feof(f_in))
    {
        ch = fgetc(f_in);
        if(true == parser.ProcessDataInput(ch))
        {
            TGNSSPosition pvt;
            static const uint16_t MAX_GNSS_SAT_CHANNEL = 26;
            TGNSSSatelliteDetail satInfo[MAX_GNSS_SAT_CHANNEL];
            uint16_t numSat;
            if(true == gnssGetPosition(&pvt))
            {
                PrintGnssSol(pvt);
            }
            if(true == gnssGetSatelliteDetails(satInfo, MAX_GNSS_SAT_CHANNEL, &numSat))
            {
                PrintGnssInfo(satInfo, numSat);
            }
            numSat = numSat;

        }
    }

}

bool gnssGetPosition(TGNSSPosition* position)
{
    return parser.GetGnssPvtData(*position);

}

bool gnssGetSatelliteDetails(TGNSSSatelliteDetail* satelliteDetails, uint16_t count, uint16_t* numSatelliteDetails)
{
    return parser.GetGnssSatData(satelliteDetails, count, numSatelliteDetails);
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


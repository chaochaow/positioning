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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include "globals.h"
#include "uBloxParser.h"


/**
 * CONFIGURATION PARAMETERS
 *
 * #required
 * GNSS_DEVICE: device name at which GNSS receiver is attached, e.g. "dev/ttyS0"
 * GNSS_BAUDRATE: baud rate of GNSS receiver at device GNSS_DEVICE, e.g. B57600
 *
 */


#define FALSE 0
#define TRUE 1

volatile int32_t g_GNSS_ublox_loop =1;
int32_t g_fd = -1;
volatile int32_t g_GpsTime = -1; 	//in milliseconds

static const uint16_t MAX_GNSS_SAT_CHANNEL = 26;

UBloxParser parser;

bool extractGnssPvtData(TGNSSPosition* gnssData);
bool extractSatelliteDetails(TGNSSSatelliteDetail* satelliteDetails, TGNSSPosition* gnssData);
bool extractTime(const uint32_t timestamp, TGNSSTime & gnss_time);

/**
 * Helper function to conventiently set GNSS status
  */
void setGNSSStatus(EGNSSStatus newStatus)
{
    static EGNSSStatus lastStatus = GNSS_STATUS_NOTAVAILABLE;
    if(newStatus != lastStatus)
    {
        lastStatus = newStatus;
        TGNSSStatus status = {0};
        status.timestamp = g_GpsTime;
        status.status = newStatus;
        status.validityBits = GNSS_STATUS_STATUS_VALID;
        updateGNSSStatus(&status);
    }
}



int open_GNSS_binary_device(const char* gps_device, unsigned int baudrate)
{
    struct termios oldtio,newtio;
    /*
    Open modem device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */
    int fd = open(gps_device, O_RDWR | O_NOCTTY );
    if (fd <0) {perror(gps_device); return -1; }

    tcgetattr(fd,&oldtio); /* save current serial port settings */
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    /*
    BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
    CRTSCTS : output hardware flow control (only used if the cable has
            all necessary lines. See sect. 7 of Serial-HOWTO)
    CS8     : 8n1 (8bit,no parity,1 stopbit)
    CLOCAL  : local connection, no modem contol
    CREAD   : enable receiving characters
    */
    newtio.c_cflag = GNSS_BAUDRATE | CS8 | CLOCAL | CREAD;

    /*
    IGNPAR  : ignore bytes with parity errors
    ICRNL   : map CR to NL (otherwise a CR input on the other computer
            will not terminate input)
    otherwise make device raw (no other input processing)
    */
    newtio.c_iflag = IGNPAR;

    /*
    Raw output.
    */
    newtio.c_oflag = 0;

    /*
    ICANON  : enable canonical input
    disable all echo functionality, and don't send signals to calling program
    */
    newtio.c_lflag = 0;

    /*
    initialize all control characters
    default values can be found in /usr/include/termios.h, and are given
    in the comments, but we don't need them here
    */
    newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
    newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    newtio.c_cc[VERASE]   = 0;     /* del */
    newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
    newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
    newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    newtio.c_cc[VEOL2]    = 0;     /* '\0' */

    /*
    now clean the modem line and activate the settings for the port
    */
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);

    /*
    terminal settings done, now handle input
    In this example, inputting a 'z' at the beginning of a line will
    exit the program.
    */

    //configure the receiver to send out the following messages
    uint8_t sCfgMsg_PVT[]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x01, 0x01, 0x07, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x19, 0xE4};
    uint8_t sCfgMsg_SAT[]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x35, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x47, 0x26};
    uint8_t sCfgMsg_MEASX[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x28, 0x4E};
    uint8_t sCfgMsg_EPH[] = {0xB5, 0x62, 0x0B, 0x31, 0x00, 0x00, 0x3C, 0xBF}; //poll all the ephemeris
    write(fd, sCfgMsg_PVT, sizeof(sCfgMsg_PVT));
    write(fd, sCfgMsg_SAT, sizeof(sCfgMsg_SAT));
    write(fd, sCfgMsg_MEASX, sizeof(sCfgMsg_MEASX));
    write(fd, sCfgMsg_EPH, sizeof(sCfgMsg_EPH));

    return fd;
}

void* loop_GNSS_binary_device(void* dev)
{
    int* p_fd = (int*)dev;
    int fd = *p_fd;
    uint32_t dataAvailMask = 0;
    uint64_t lastSatDataTow = 604800001;
    uint64_t lastPvtDataTow = 604800001;
    int64_t lastEphPollTime = - 60000;

    TGNSSPosition pvtData;
    TGNSSSatelliteDetail satInfo[MAX_GNSS_SAT_CHANNEL];
    uint16_t numSat, count;
    fd_set readfs;
    int maxfd;
    uint8_t ch;

    while(g_GNSS_ublox_loop)
    {
        int res;
        struct timeval timeout;
        timeout.tv_usec = 0;
        timeout.tv_sec = 2;
        FD_SET(fd, &readfs);
        maxfd = fd+1;
        res = select(maxfd, &readfs, NULL, NULL, &timeout);
        if(res == -1)
        {
            g_GNSS_ublox_loop = false;
        }
        else if(FD_ISSET(fd, &readfs))
        {
            res = read(fd, &ch, 1);

            if(true == parser.ProcessDataInput(ch))
            {
                uint32_t dataType = parser.GetUBloxDataType() ;

                if(UBLOX_PVT_DATA_READY == dataType)
                {
                    if(true == extractGnssPvtData(&pvtData))
                    {

                        g_GpsTime = pvtData.timestamp;
                        TGNSSTime gnss_time = { 0 };
                        if(true == extractTime(g_GpsTime, gnss_time))
                        {
                            updateGNSSTime(&gnss_time, 1);
                        }

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
                    if(true == extractSatelliteDetails(satInfo, &pvtData))
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
                    setGNSSStatus(GNSS_STATUS_AVAILABLE);
                    updateGNSSPosition(&pvtData, 1);
                    updateGNSSSatelliteDetail(satInfo, pvtData.visibleSatellites);

                }
            }

        }
    }

    close(fd);
}

pthread_t g_thread;


extern bool gnssInit()
{
    iGnssInit();

    setGNSSStatus(GNSS_STATUS_INITIALIZING);
    g_fd = open_GNSS_binary_device(GNSS_DEVICE, GNSS_BAUDRATE);
    if (g_fd >=0)
    {
        pthread_create(&g_thread, NULL, loop_GNSS_binary_device, &g_fd);
        return true;
    }
    else
    {
        setGNSSStatus(GNSS_STATUS_FAILURE);
        return false;
    }
}


bool extractGnssPvtData(TGNSSPosition* gnssData)
{
    UBloxPVTStruct * pPVT = NULL;
    if(true == parser.GetGnssPvtData(&pPVT))
    {
        memset(gnssData, 0, sizeof(TGNSSPosition));
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

bool extractSatelliteDetails(TGNSSSatelliteDetail* satelliteDetails, TGNSSPosition* gnssData)
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

bool extractTime(uint32_t timestamp, TGNSSTime & gnss_time)
{
    static const uint32_t MSINADAY = 86400000;
    if(timestamp >=0)
    {
        int day = timestamp / MSINADAY;
        timestamp -= day * MSINADAY;
        gnss_time.hour = timestamp / 3600000;
        timestamp -= gnss_time.hour * 3600000;
        gnss_time.minute = timestamp / 60000;
        timestamp -= gnss_time.minute * 60000;
        gnss_time.second = timestamp / 1000;
        gnss_time.ms = timestamp - gnss_time.second*1000;
        gnss_time.validityBits |= GNSS_TIME_TIME_VALID;
        return true;
    }
    else
    {
        gnss_time.validityBits = 0;
        return false;
    }

}

void gnssGetVersion(int *major, int *minor, int *micro)
{
    if(major)
    {
        *major = GENIVI_GNSS_API_MAJOR;
    }

    if (minor)
    {
        *minor = GENIVI_GNSS_API_MINOR;
    }

    if (micro)
    {
        *micro = GENIVI_GNSS_API_MICRO;
    }
}

bool gnssSetGNSSSystems(uint32_t activate_systems)
{
    //this API needs to be implemented in the future.
    return false;
}

extern bool gnssDestroy()
{
    g_GNSS_ublox_loop = 0;
    pthread_join(g_thread, NULL);

    iGnssDestroy();

    return true;
}




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

void main(int argc, char * argv[])
{

    if(argc < 2)
    {
        printf("Please provide file input\n");
        return;
    }

    FILE * f_in = fopen(argv[1], "rb");
    uint8_t ch;

    UBloxParser parser;
    while(!feof(f_in))
    {
        ch = fgetc(f_in);
        parser.ProcessDataInput(ch);
    }

}
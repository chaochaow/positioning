###########################################################################
# @licence app begin@
# SPDX-License-Identifier: MPL-2.0
#
# Component Name: EnhancedPositionService
#
# Author: Marco Residori
#
# Copyright (C) 2014, XS Embedded GmbH
# 
# License:
# This Source Code Form is subject to the terms of the
# Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with
# this file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
# Update (2014/12/02) : Philippe Colliot <philippe.colliot@mpsa.com>,
#				PSA Peugeot Citroen
#		- introduce debug flag to disable verbosity
# @licence end@
###########################################################################

project(uBloxParser)
cmake_minimum_required(VERSION 2.6.0)

message(STATUS "uBloxParser")

set(gnss-service_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../../gnss-service/api")

include_directories(${gnss-service_INCLUDE_DIRS})

add_executable(uBloxParser
    main.cpp
    uBloxParser.cpp
    uBloxParser.h
)

message(STATUS "GNSS_SERVICE_LIBRARIES: " ${gnss-service_LIBRARIES})




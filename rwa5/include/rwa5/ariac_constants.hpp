/*
* Constants belonging to ARIAC competition
* See COPYRIGHT file at the top of the source tree.
*
* Licensed to the Apache Software Foundation (ASF) under one
* or more contributor license agreements.  See the NOTICE file
* distributed with this work for additional information
* regarding copyright ownership.  The ASF licenses this file
* to you under the Apache License, Version 2.0 (the
* "License"); you may not use this file except in compliance
* with the License.  You may obtain a copy of the License at

*  http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing,
* software distributed under the License is distributed on an
* "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
* KIND, either express or implied.  See the License for the
* specific language governing permissions and limitations
* under the License.
*/

/**
 * @file ariac_constants.hpp
 * @brief araic specific constants
 * @author Tej Kiran
 */

#pragma once

#include <string>

/*! \namespace ARIAC_FRAME ariac_constants.hpp "inc/ARIAC_FRAME.h"
 *  \brief This name space provides constants representing ariac tf frame names.
 *
 * Use type ARIAC_FRAME::NAME to specify where these constants are passed
 */
namespace ARIAC_FRAME{
    typedef std::string NAME;

    const std::string R_BIN_CAMERA_FRAME = "right_bins_camera_frame";
    const std::string L_BIN_CAMERA_FRAME = "left_bins_camera_frame";

    const std::string KTS1_BIN_CAMERA_FRAME = "kts1_bins_camera_frame";
    const std::string KTS2_BIN_CAMERA_FRAME = "kts2_bins_camera_frame";

    const std::string WORLD = "world";
};



/****************************************************************************
* Copyright (c) 2014, Paul Riseborough All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* Neither the name of the {organization} nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

/**
 * @file estimator_utilities.h
 *
 * Estimator support utilities.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include <math.h>
#include <stdint.h>

#define GRAVITY_MSS 9.80665f
#define deg2rad 0.017453292f
#define rad2deg 57.295780f
#define earthRate 0.000072921f
#define earthRadius 6378145.0
#define earthRadiusInv  1.5678540e-7

class Vector3f_px4
{
public:
    float x;
    float y;
    float z;

    Vector3f_px4(float a=0.0f, float b=0.0f, float c=0.0f) :
    x(a),
    y(b),
    z(c)
    {}

    float length() const;
    void zero();
};

class Mat3f
{
private:
public:
    Vector3f_px4 x; 
    Vector3f_px4 y;
    Vector3f_px4 z;

    Mat3f()
{
	  x.x=1.0; x.y=0.0; x.z=0.0;
    y.x=0.0; y.y=1.0; y.z=0.0;
	  z.x=0.0; z.y=0.0; z.z=1.0;
	
}


    void identity();
    Mat3f transpose() const;
};

Vector3f_px4 operator*(const float sclIn1, const Vector3f_px4 &vecIn1);
Vector3f_px4 operator+(const Vector3f_px4 &vecIn1, const Vector3f_px4 &vecIn2);
Vector3f_px4 operator-(const Vector3f_px4 &vecIn1, const Vector3f_px4 &vecIn2);
Vector3f_px4 operator*(const Mat3f &matIn, const Vector3f_px4 &vecIn);
Mat3f operator*(const Mat3f &matIn1, const Mat3f &matIn2);
Vector3f_px4 operator%(const Vector3f_px4 &vecIn1, const Vector3f_px4 &vecIn2);
Vector3f_px4 operator*(const Vector3f_px4 &vecIn1, const float sclIn1);
Vector3f_px4 operator/(const Vector3f_px4 &vec, const float scalar);

enum GPS_FIX {
     GPS_FIX_NOFIX = 0,
     GPS_FIX_2D = 2,
     GPS_FIX_3D = 3
};

struct ekf_status_report {
    bool error;
    bool velHealth;
    bool posHealth;
    bool hgtHealth;
    bool velTimeout;
    bool posTimeout;
    bool hgtTimeout;
    bool imuTimeout;
    bool onGround;
    bool staticMode;
    bool useCompass;
    bool useAirspeed;
    uint32_t velFailTime;
    uint32_t posFailTime;
    uint32_t hgtFailTime;
    float states[32];
    unsigned n_states;
    bool angNaN;
    bool summedDelVelNaN;
    bool KHNaN;
    bool KHPNaN;
    bool PNaN;
    bool covarianceNaN;
    bool kalmanGainsNaN;
    bool statesNaN;
    bool gyroOffsetsExcessive;
    bool covariancesExcessive;
    bool velOffsetExcessive;
};


void ekf_debug(const char *fmt, ...);

void calcvelNED(float (&velNEDr)[3], float gpsCourse, float gpsGndSpd, float gpsVelD);

void calcposNED(float (&posNEDr)[3], double lat, double lon, float hgt, double latReference, double lonReference, float hgtReference);

void calcLLH(float posNEDi[3], double &lat, double &lon, float &hgt, double latRef, double lonRef, float hgtRef);

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "geo_pos_conv.h"
#include <stdio.h>
#include <iostream>
using namespace std;


const double PI = 3.1415926535897932384; ///< PI
const double D2R = 0.017453292519943; 
const double R2D = 57.29577951308232; 
static int BLH2xyz(double B,double L,double H,double& x, double& y, double& z)
{
   // B = B*D2R; L = L*D2R;
    const double a = 6378137;
    const double e2 = 0.0066943799013;
    double N = a/sqrt(1-e2*pow(sin(B),2));
    x = (N+H)*cos(B)*cos(L);
    y = (N+H)*cos(B)*sin(L);
    z = (N*(1-e2)+H) * sin(B);
    return 0;
}


geo_pos_conv::geo_pos_conv()
    : m_x0(0)
    , m_y0(0)
    , m_z0(0)
    , m_x(0)
    , m_y(0)
    , m_z(0)
    , m_lat(0)
    , m_lon(0)
    , m_h(0)
    , m_OLat(24.599862)
    , m_OLo(117.9776288)
    , m_initialized(false)
{
    m_OLat = m_OLat*D2R;
    m_OLo = m_OLo*D2R;
    BLH2xyz(m_OLat, m_OLo, 0, m_x0, m_y0, m_z0);
}

double geo_pos_conv::x() const
{
  return m_x;
}

double geo_pos_conv::y() const
{
  return m_y;
}

double geo_pos_conv::z() const
{
  return m_z;
}

void geo_pos_conv::set_origin(double lat, double lon)
{
  m_OLo = lat*D2R;
  m_OLat = lon*D2R;
  BLH2xyz(m_OLat, m_OLo, 0, m_x0, m_y0, m_z0);
  m_initialized = true;
}


void geo_pos_conv::set_xyz(double cx, double cy, double cz)
{
  m_x = cx;
  m_y = cy;
  m_z = cz;
  conv_xyz2llh();
}

void geo_pos_conv::set_llh_nmea_degrees(double latd, double lond, double h)
{
  if (!m_initialized) 
	return;
  double lat, lad, lod, lon;
  // 1234.56 -> 12'34.56 -> 12+ 34.56/60
#if 0
  lad = floor(latd / 100.);
  lat = latd - lad * 100.;
  lod = floor(lond / 100.);
  lon = lond - lod * 100.;
#endif
 // printf("set llh: lat:%lf, lon:%lf, mOriLon:%lf, mOriLat:%lf \n", latd, lond, m_OLat, m_OLo);
  // Changing Longitude and Latitude to Radians
  m_lat = latd*D2R;//(lad + lat / 60.0) * M_PI / 180;
  m_lon = lond*D2R;//(lod + lon / 60.0) * M_PI / 180;
  m_h = h;

  conv_llh2xyz();
}

void geo_pos_conv::llh_to_xyz(double lat, double lon, double ele)
{
  m_lat = lat;
  m_lon = lon;
  m_h = ele;

  conv_llh2xyz();
}

void geo_pos_conv::conv_llh2xyz(void)
{
  double xc, yc, zc;
  double x,y,z;
  
  BLH2xyz(m_lat, m_lon, m_h, xc, yc, zc);
  
  xc -= m_x0; 
  yc -= m_y0;
  zc -= m_z0;

  x = -sin(m_lon)*xc + cos(m_lon)*yc;
  y = -sin(m_lat)*cos(m_lon)*xc + (-sin(m_lon)*sin(m_lat))*yc + cos(m_lat)*zc;
  z = cos(m_lat)*cos(m_lon)*xc + cos(m_lat)*sin(m_lon)*yc + sin(m_lat)*zc;

  m_x = y;
  m_y = -x;

}

void geo_pos_conv::conv_xyz2llh(void)
{
  // n/a
}


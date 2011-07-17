/*
  SirfGPS - a small GPS library for Arduino providing basic Sirf Binary parsing
  by Mike Cochrane http://mikenz.geek.nz
  
  Interface based on the excelent "TinyGPS" by Mikal Hart 
  http://arduiniana.org/libraries/tinygps/
  Copyright (C) 2008-2011 Mikal Hart
  
  SirfBinary parse based on "SiRF Binary GPS driver for ArduPilot and ArduPilotMega" by Michael Smith.
  http://code.google.com/p/arducopter/source/browse/trunk/libraries/AP_GPS/AP_GPS_SIRF.h
  
  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SirfGPS_h
#define SirfGPS_h

#include "WProgram.h"

#define _GPS_VERSION 1 // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001

class SirfGPS
{
  public:
    SirfGPS();
    bool encode(byte c); // process one character received from GPS
    
    // lat/long in thousandths of a degree and age of fix in milliseconds
    inline void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0)
    {
        if (latitude) *latitude = _latitude;
        if (longitude) *longitude = _longitude;
        if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ? 
            GPS_INVALID_AGE : millis() - _last_position_fix;
    }
    
    inline void f_get_position(float *latitude, float *longitude, unsigned long *fix_age = 0) {
        long lat, lon;
        get_position(&lat, &lon, fix_age);
        *latitude = lat / 10000000.0;
        *longitude = lon / 10000000.0;
    }

    // signed altitude in centimeters
    inline long altitude() { return _altitude; }

    // course in last full GPRMC sentence in 100th of a degree
    inline unsigned long course() { return _course; }
    
    // speed in last full GPRMC sentence in 100ths of a knot
    unsigned long speed() { return _speed; }

    // date as ddmmyy, time as hhmmsscc, and age in milliseconds
    inline void get_datetime(unsigned long *date, unsigned long *time, unsigned long *fix_age = 0)
    {
        if (date) *date = _date;
        if (time) *time = _time;
        if (fix_age) *fix_age = _last_time_fix == GPS_INVALID_FIX_TIME ? 
            GPS_INVALID_AGE : millis() - _last_time_fix;
    }

    inline void crack_datetime(int *year, byte *month, byte *day, 
      byte *hour, byte *minute, byte *second, byte *hundredths = 0, unsigned long *fix_age = 0)
    {
      unsigned long date, time;
      get_datetime(&date, &time, fix_age);
      if (year) 
      {
        *year = date % 100;
        *year += *year > 80 ? 1900 : 2000;
      }
      if (month) *month = (date / 100) % 100;
      if (day) *day = date / 10000;

      if (hour) *hour = time / 1000000;
      if (minute) *minute = (time / 10000) % 100;
      if (second) *second = (time / 100) % 100;
      if (hundredths) *hundredths = time % 100;
    }

    inline float f_altitude()    { return altitude() / 100.0; }
    inline float f_course()      { return course() / 100.0; }
    inline float f_speed_knots() { return speed() / 100.0; }
    inline float f_speed_mph()   { return _GPS_MPH_PER_KNOT * f_speed_knots(); }
    inline float f_speed_mps()   { return _GPS_MPS_PER_KNOT * f_speed_knots(); }
    inline float f_speed_kmph()  { return _GPS_KMPH_PER_KNOT * f_speed_knots(); }

    static int library_version() { return _GPS_VERSION; }

    enum {GPS_INVALID_AGE = 0xFFFFFFFF, GPS_INVALID_ANGLE = 999999999, GPS_INVALID_ALTITUDE = 999999999, GPS_INVALID_DATE = 0,
      GPS_INVALID_TIME = 0xFFFFFFFF, GPS_INVALID_SPEED = 999999999, GPS_INVALID_FIX_TIME = 0xFFFFFFFF};


private:
    // properties
    unsigned long _time;
    unsigned long _date;
    long _latitude, _longitude, _altitude;
    unsigned long _speed, _course;

    unsigned long _last_time_fix, _last_position_fix;

    // parsing state variables
    uint8_t                 _step;
    uint16_t                _checksum;
    uint16_t                _payload_length;
    uint16_t                _payload_counter;
    uint8_t                 _msg_id;
    bool _gps_data_good;
    
    struct sirf_geonav {
            uint16_t        fix_invalid;
            uint16_t        fix_type;
            uint16_t        week;
            uint32_t        time;
            uint16_t        year;
            uint8_t         month;
            uint8_t         day;
            uint8_t         hour;
            uint8_t         minute;
            uint16_t        second;
            uint32_t        satellites_used;
            int32_t         latitude;
            int32_t         longitude;
            int32_t         altitude_ellipsoid;
            int32_t         altitude_msl;
            int8_t          map_datum;
            int16_t         ground_speed;
            int16_t         ground_course;
            int16_t         res1;
            int16_t         climb_rate;
            uint16_t        heading_rate;
            uint32_t        horizontal_position_error;
            uint32_t        vertical_position_error;
            uint32_t        time_error;
            int16_t         horizontal_velocity_error;
            int32_t         clock_bias;
            uint32_t        clock_bias_error;
            int32_t         clock_drift;
            uint32_t        clock_drift_error;
            uint32_t        distance;
            uint16_t        distance_error;
            uint16_t        heading_error;
            uint8_t         satellites;
            uint8_t         hdop;
            uint8_t         mode_info;
    };

    // Message buffer
    union {
            sirf_geonav             nav;
            uint8_t                 bytes[];
    } _buffer;
    
    inline long _swapl(const void *bytes) {
        const uint8_t *b = (const uint8_t *)bytes;
        union {
                long    v;
                uint8_t b[4];
        } u;

        u.b[0] = b[3];
        u.b[1] = b[2];
        u.b[2] = b[1];
        u.b[3] = b[0];

        return(u.v);
    }

    inline uint16_t _swapi(const void *bytes) {
            const uint8_t   *b = (const uint8_t *)bytes;
            union {
                    uint16_t v;
                    uint8_t b[2];
            } u;
    
            u.b[0] = b[1];
            u.b[1] = b[0];
    
            return(u.v);
    }    

    bool                    _parse_gps(void);
    void                    _accumulate(uint8_t val);
    
};


#endif

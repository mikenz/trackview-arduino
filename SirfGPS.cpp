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

#include "WProgram.h"
#include "SirfGPS.h"

#define PREAMBLE1  0xa0
#define PREAMBLE2  0xa2
#define POSTAMBLE1 0xb0
#define POSTAMBLE2 0xb3
#define MSG_GEONAV 0x29

SirfGPS::SirfGPS() {
    _step = 0;
}

bool SirfGPS::_parse_gps(void) {
    switch(_msg_id) {
        case MSG_GEONAV:
            // ddmmyy
            _date            = (unsigned long)_buffer.nav.day * 10000;
            _date           += _buffer.nav.month * 100;
            _date           += _swapi(&_buffer.nav.year) % 100;
            
            // hhmmsscc
            _time            = (unsigned long)_buffer.nav.hour * 1000000;
            _time           += (unsigned long)_buffer.nav.minute * 10000;
            _time           += _swapi(&_buffer.nav.second);
            
            _latitude        = _swapl(&_buffer.nav.latitude);
            _longitude       = _swapl(&_buffer.nav.longitude);
            _altitude        = _swapl(&_buffer.nav.altitude_msl);
            _speed           = _swapi(&_buffer.nav.ground_speed);
            _course          = _swapi(&_buffer.nav.ground_course);
            
            _last_time_fix   = millis();

            return (0 == _buffer.nav.fix_invalid);
    }
    return false;
}

void SirfGPS::_accumulate(uint8_t val) {
    _checksum = (_checksum + val) & 0x7fff;
}

bool SirfGPS::encode(byte c) {
    switch (_step) {
        case 1:
            if (c == PREAMBLE2) {
                /* We got a complete preamble */
                _step++;
                break;
            }
                
            /* Wasn't expected 2nd preamble char, maybe the first? */
            _step = 0;

        case 0:
            if (c == PREAMBLE1) {
                /* We got the first preamble byte */
                _step++;
            }
            break;

        case 2:
            /* PDU Length 1 */
            _step++;
            _payload_length = (uint16_t)c << 8;
            break;

        case 3:
            /* PDU Length 2 */
            _step++;
            _payload_length |= c;
            _payload_counter = 0;
            _checksum = 0;
            _msg_id = 0;
            break;            

        case 4:
            /* PDU Type */
            _step++;
            _accumulate(c);
            _payload_length--;
            switch (c) {
                case MSG_GEONAV:
                    if (_payload_length == sizeof(sirf_geonav)) {
                        _msg_id = c;
                    }
                    break;
            }
            break; 
    
        case 5:
            if (_msg_id == MSG_GEONAV) {
                /* We want this data */
                _accumulate(c);
                _buffer.bytes[_payload_counter] = c;
                if (++_payload_counter == _payload_length) {
                    _step++;
                }
            } else if (++_payload_counter == _payload_length) {
                /* We don't want this data */
                _step = 0;
            }
            break;

        case 6:
            /* Verify Checksum 1 */
            _step++;
            if ((_checksum >> 8) != c) {
                /* GPS_SIRF: checksum error */
                _step = 0;
            }
            break;
            
        case 7:
            /* Verify Checksum 2 */
            _step = 0;
            if ((_checksum & 0xff) != c) {
                /* GPS_SIRF: checksum error */
                break;
            }
            if (_msg_id == MSG_GEONAV) {
                // Parse the new GPS packet
                _gps_data_good = _parse_gps();
                return true;
            }
            break;
    }        
    return false;
}


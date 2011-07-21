// DIY Streetview / Trackview camera controller

/* ============================================
Controller code is placed under the MIT license
Copyright (c) 2011 Mike Cochrane - mikenz.geek.nz

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/*** Configuration options ***/

/* Sensors */
#define ENABLE_FREEIMU  // FreeIMU
//#define ENABLE_TINYGPS  // Tiny GPS serial NMEA
#define ENABLE_SIRFGPS  // Sirf Binary GPS serial
#define ENABLE_HMC6352  // HMC6352 I2C compass
#define ENABLE_BMP085   // BMP085 I2C Barometer
#define ENABLE_LDR      // LDR for relative light level
#define ENABLE_BEEP     // Piezo Speaker for audio feedback
#define ENABLE_LCD12864 // Serial LCD12864 128x64 Grahpical display
#define ENABLE_LEDBUTTONS // Go/Reset/Stop buttons

/* Cameras */
#define NUM_SLAVES      5
unsigned char GOPRO_SLAVES[] = {30, 31, 32, 33, 34};
#define PHOTO_DELAY     500 // milli seconds between finishing a photo and starting the next

/* GPS */
#define GPS_SERIAL      Serial3
#define GPS_BAUD        38400

/* Pins */
#define GOPRO_TRIG      27
#define GOPRO_ID2       28
#define GOPRO_ID3       29
#define SS_PIN          53 // SPI pin
#define LDR_PIN         A15
#define BEEP_PIN        4  // Any PWM pin you choose
#define GOLED_PIN       22
#define RSTLED_PIN      23
#define STPLED_PIN      24
#define GOBTN_PIN       39
#define RSTBTN_PIN      40
#define STPBTN_PIN      38

/*** End of config ***/


/* Sd Card */
#include <avr/eeprom.h>
#include "SDCARDmodded.h"
#include <SPI.h>
byte buffer[512];
uint32_t sector = 0;
int bufferPos = 0;

/* Global Variables */
bool goprotrigger = false;
uint32_t photo = 0;
unsigned int retryCount = 0;
uint32_t lastPhoto = 0;

/* I2C */
#if defined(ENABLE_FREEIMU) || defined(ENABLE_BMP085) || defined(ENABLE_HMC6352)
    #include <Wire.h>
#endif

/* GPS */
#ifdef ENABLE_TINYGPS
    #include <TinyGPS.h>
    TinyGPS gps;
    bool newGPSData = false;
    bool timeUpdated = false;
    int year;
    byte month, day, hour, minute, second, hundredths;
    float lat, lon, alt, course, speed;
    unsigned long age;
#endif /* ENABLE_TINYGPS */

#ifdef ENABLE_SIRFGPS
    #include "SirfGPS.h"
    SirfGPS gps;
    bool newGPSData = false;
    bool timeUpdated = false;
    int year;
    byte month, day, hour, minute, second, hundredths;
    float lat, lon, alt, course, speed;
    unsigned long age;
#endif /* ENABLE_SIRFGPS */


/* LCD */
#ifdef ENABLE_LCD12864
    #include <avr/pgmspace.h>
    #include "LCD12864RSPI.h"
    #include "font.h";
    #include "floatToString.h"
    #define FONT_HEIGHT 9
    char line[70];
    unsigned char screenBuffer[1024];
    uint32_t lastTime = 0;
    bool fullRedraw = false;
#endif /* ENABLE_LCD12864 */

/* IMU */
#ifdef ENABLE_FREEIMU
    #include <ADXL345.h>
    #include <HMC58X3.h>
    #include <ITG3200.h>
    #include <FreeIMU.h>
    #include <CommunicationUtils.h>
    float imu_val[9];
    float ypr[3];
    FreeIMU my3IMU = FreeIMU();
#endif /* ENABLE_FREEIMU */

/* Compass */
#ifdef ENABLE_HMC6352
    #include <hmc6352.h>
    float heading;
#endif /* ENABLE_HMC6352 */

/* Barometer */
#ifdef ENABLE_BMP085
    #include <BMP085.h>
    BMP085 dps = BMP085();
    long Temperature = 0, Pressure = 0, Altitude = 0;
    uint32_t nextBmp085 = 0;
#endif /* ENABLE_BMP085 */

void setup()
{
    /* Setup the LCD */
    #ifdef ENABLE_LCD12864
        LCDA.Initialise();
        LCDClearScreen();
        LCDPrintString(0, 0, "Trackview", true);
        lastTime = millis() - 1000;
    #endif /* ENABLE_LCD12864 */

    /* Start GPS */
    #ifdef ENABLE_TINYGPS
        GPS_SERIAL.begin(GPS_BAUD);
    #endif /* ENABLE_TINYGPS */
    #ifdef ENABLE_SIRFGPS
        /* Switch to 9600 in SIRF Binary mode */
        GPS_SERIAL.begin(GPS_BAUD);  
        GPS_SERIAL.print("$PSRF100,0,9600,8,1,0*0C\r\n");
        GPS_SERIAL.end();  
        GPS_SERIAL.begin(9600);  
    #endif /* ENABLE_SIRFGPS */

    
    /* I2C */
    #if defined(ENABLE_FREEIMU) || defined(ENABLE_BMP085) || defined(ENABLE_HMC6352)
        Wire.begin();
    #endif

    /* Start IMU */
    #ifdef ENABLE_FREEIMU
        #ifdef ENABLE_LCD12864
            LCDPrintString(3, 0, "Starting IMU       ", true);
        #endif /* ENABLE_LCD12864 */
        delay(100);
        my3IMU.init(false);
        delay(500);
        #ifdef ENABLE_LCD12864
            LCDPrintString(3, 0, "Started IMU       ", true);
        #endif /* ENABLE_LCD12864 */
    #endif /* ENABLE_FREEIMU */

    /* Start Barometer */
    #if defined(ENABLE_BMP085) && !(defined(ENABLE_TINYGPS) || defined(ENABLE_SIRFGPS))
        dps.init(); // Initialise for relative altitude
        dps.setMode(MODE_ULTRA_LOW_POWER);
        nextBmp085 = millis() + 1000;
        delay(500);
    #endif /* ENABLE_BMP085 */

    /* Continue where we were */
    sector = eeprom_read_dword((uint32_t *) 0x01);
    photo  = eeprom_read_dword((uint32_t *) 0x05);
    #ifdef ENABLE_LCD12864
        sprintf(line, "Photos: %04d   ", photo);
        LCDPrintString(0, 0, line, true);
    #endif /* ENABLE_LCD12864 */

    /* Wait for GPS initial data */
    #if defined(ENABLE_TINYGPS) || defined(ENABLE_SIRFGPS)
        #ifdef ENABLE_LCD12864
            LCDPrintString(3, 0, "Waiting for GPS       ", true);
        #endif /* ENABLE_LCD12864 */
        while (!newGPSData) {
            while (GPS_SERIAL.available()) {
                if (gps.encode(GPS_SERIAL.read())) {
                    newGPSData = true;
                }
            }
        }
        #ifdef ENABLE_LCD12864
            LCDPrintString(3, 0, "                      ", true);
        #endif /* ENABLE_LCD12864 */

        #if defined(ENABLE_BMP085) && !defined(ENABLE_LEDBUTTONS)
            /* Initialise with actual starting altitude */
            dps.init(MODE_ULTRA_LOW_POWER, gps.altitude(), true);
            delay(500);
        #endif /* ENABLE_BMP085 */
    #endif /* ENABLE_TINYGPS || ENABLE_SIRFGPS */

    /* Go Pro Cameras */
    pinMode(GOPRO_TRIG, OUTPUT);
    pinMode(GOPRO_ID2, OUTPUT);
    pinMode(GOPRO_ID3, OUTPUT);

    /* Input Pins */
    for (int i = 0; i < NUM_SLAVES; i++) {
        pinMode(GOPRO_SLAVES[i], INPUT);
    }

    /* Set initial state */
    digitalWrite(GOPRO_TRIG, HIGH);
    digitalWrite(GOPRO_ID2, HIGH);
    digitalWrite(GOPRO_ID3, LOW);

    /* Beep */
    #ifdef ENABLE_BEEP
        tone(BEEP_PIN, 2200, 200);
        delay(250);
        tone(BEEP_PIN, 2200, 200);
    #endif /* ENABLE_BEEP */

    #ifdef ENABLE_LEDBUTTONS
        pinMode(GOLED_PIN, OUTPUT);
        pinMode(RSTLED_PIN, OUTPUT);
        pinMode(STPLED_PIN, OUTPUT);
        pinMode(GOBTN_PIN, INPUT);
        pinMode(RSTBTN_PIN, INPUT);
        pinMode(STPBTN_PIN, INPUT);

        digitalWrite(GOLED_PIN, HIGH);
        digitalWrite(RSTLED_PIN, HIGH);
        digitalWrite(STPLED_PIN, LOW);

        #ifdef ENABLE_LCD12864
            LCDPrintString(3, 0, "Green to start       ", true);
            LCDPrintString(4, 0, "Orange to reset photo", true);
        #endif /* ENABLE_LCD12864 */

        while(true) {
            if (digitalRead(RSTBTN_PIN) == 1) {
                /* Reset */
                photo = 0;
                sector = 0;
                eeprom_update_dword((uint32_t *) 0x01, sector);
                eeprom_update_dword((uint32_t *) 0x05, photo);
                #ifdef ENABLE_LCD12864
                    sprintf(line, "Photos: %04d   ", photo);
                    LCDPrintString(0, 0, line, true);
                    LCDPrintString(4, 0, "                     ", true);
                #endif /* ENABLE_LCD12864 */
                digitalWrite(RSTLED_PIN, LOW);
            }
            if (digitalRead(GOBTN_PIN) == 1) {
                digitalWrite(GOLED_PIN, LOW);
                digitalWrite(RSTLED_PIN, LOW);
                digitalWrite(STPLED_PIN, HIGH);
                break;
            }

            /* Feed GPS and IMU */
            usefulDelay(1);
        }

        #if defined(ENABLE_BMP085)
            /* Initialise with GPS altitude */
            dps.init(MODE_ULTRA_LOW_POWER, gps.altitude(), true);
            usefulDelay(500);
        #endif /* ENABLE_BMP085 */
    #endif /* ENABLE_LEDBUTTONS */

    /* Start timer */
    lastPhoto = millis();
}

void loop()
{
    #ifdef ENABLE_LEDBUTTONS
        #ifdef ENABLE_LCD12864
            if (digitalRead(GOBTN_PIN) == 1) {
                if (!fullRedraw) {
                    sprintf(line, "Photos: %04d    ", photo);
                    LCDA.Initialise();
                    LCDA.DisplayString(0, 0, line, 16);
                    LCDA.DisplayString(1, 0, line, 16);
                    LCDA.DisplayString(2, 0, line, 16);
                    LCDA.DisplayString(3, 0, line, 16);
                    fullRedraw = true;
                }
                lastTime = millis();
            }
        #endif /* ENABLE_LCD12864 */

        #if defined(ENABLE_BMP085) && (defined(ENABLE_TINYGPS) || defined(ENABLE_SIRFGPS))
            if (digitalRead(RSTBTN_PIN) == 1) {
                /* Re initialise barometer with GPS altitude */
                dps.init(MODE_ULTRA_LOW_POWER, gps.altitude(), true);
                digitalWrite(RSTLED_PIN, HIGH);
                usefulDelay(500);
                digitalWrite(RSTLED_PIN, LOW);
            }
        #endif /* ENABLE_BMP085 */

        if (!goprotrigger && digitalRead(STPBTN_PIN) == 1) {
            /* Stop */
            digitalWrite(STPLED_PIN, LOW);

            /* Back to normal state */
            digitalWrite(GOPRO_ID2, HIGH);
            digitalWrite(GOPRO_ID3, LOW);

            /* Flush SD card write buffer */
            byte null[1] = { 0 };
            while (bufferPos) {
                addToSDCard(null, 1);
            }

            /* Remember where we were up to */
            eeprom_update_dword((uint32_t *) 0x01, sector);
            eeprom_update_dword((uint32_t *) 0x05, photo);

            /* Clear screen */
            #ifdef ENABLE_LCD12864
                LCDPrintString(1, 0, "                     ", true);
                LCDPrintString(2, 0, "                     ", true);
                LCDPrintString(3, 0, "       STOPPED       ", true);
                LCDPrintString(4, 0, "                     ", true);
                LCDPrintString(5, 0, "                     ", true);
                LCDPrintString(6, 0, "                     ", true);
            #endif /* ENABLE_LCD12864 */

            while (true) {};
        }
    #endif /* ENABLE_LEDBUTTONS */

    /* GPS */
    #if defined(ENABLE_TINYGPS) || defined(ENABLE_SIRFGPS)
        /* Feed the TinyGPS with new data from GPS */
        usefulDelay(0);

        /* Get time */
        gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

        byte datedata[6] = {byte(month), byte(day), byte(hour), byte(minute), byte(second), byte(hundredths)};
        stringToSDCard("D");
        intToSDCard(year);
        addToSDCard(datedata, 6);
        longToSDCard(age);

        if (newGPSData) {
            /* Write new GPS data to SD card */
            gps.f_get_position(&lat, &lon, &age);
            alt = gps.f_altitude();
            course = gps.f_course();
            speed = gps.f_speed_knots();

            stringToSDCard("G");
            floatToSDCard(lat);
            floatToSDCard(lon);
            floatToSDCard(alt);
            floatToSDCard(course);
            floatToSDCard(speed);
            longToSDCard(age);

            newGPSData = false;
        }
    #endif /* ENABLE_TINYGPS || ENABLE_SIRFGPS */

    /* Update the IMU */
    #ifdef ENABLE_FREEIMU
        usefulDelay(0);

        stringToSDCard("I");
        floatToSDCard(ypr[0]);
        floatToSDCard(ypr[1]);
        floatToSDCard(ypr[2]);
    #endif /* ENABLE_FREEIMU */

    /* Update heading */
    #ifdef ENABLE_HMC6352
        hmc6352.wake();
        heading = hmc6352.getHeading();
        hmc6352.sleep();

        stringToSDCard("C");
        floatToSDCard(heading);
    #endif /* ENABLE_HMC6352 */

    /* Update barometer */
    #ifdef ENABLE_BMP085
        if (millis() > nextBmp085) {
            dps.getTemperature(&Temperature);
            dps.getPressure(&Pressure);
            dps.getAltitude(&Altitude);

            stringToSDCard("B");
            longToSDCard(Temperature);
            longToSDCard(Pressure);
            longToSDCard(Altitude);

            // 1Hz Barometer samples as they are time consuming
            nextBmp085 = millis() + 1000;
        }
    #endif /* ENABLE_BMP085 */

    usefulDelay(0);

    #ifdef ENABLE_LDR
        int ldr = analogRead(LDR_PIN);

        stringToSDCard("L");
        intToSDCard(ldr);
    #endif /* ENABLE_LDR */

    /* Update LCD Display */
    #ifdef ENABLE_LCD12864
        /* Update the display every second */
        if (millis() - lastTime > 1000) {
            int len, pos = 0;

            #if defined(ENABLE_TINYGPS) || defined(ENABLE_SIRFGPS)
                /* Update time */
                sprintf(line, "%02d:%02d\0", hour, minute);
                LCDPrintString(0, 16, line, true);

                /* Update GPS Data */
                float temp = lat;
                len = floatToString(line, int(temp), 0);
                LCDPrintString(2, pos, line, false);
                pos += len - 1;
                LCDPrintString(2, pos - 1, "@", false);
                temp *= -1;

                temp = (temp - int(temp)) * 60;
                len = floatToString(line, int(temp), 0);
                LCDPrintString(2, pos, line, false);
                pos += len - 1;
                LCDPrintString(2, pos - 1, "'", false);

                temp = (temp - int(temp)) * 60;
                len = floatToString(line, int(temp), 0);
                LCDPrintString(2, pos, line, false);
                pos += len - 1;
                LCDPrintString(2, pos - 1, "\"", false);
                LCDPrintString(2, pos, " ", false);
                pos += 1;

                temp = lon;
                len = floatToString(line, int(temp), 0);
                LCDPrintString(2, pos, line, false);
                pos += len - 1;
                LCDPrintString(2, pos - 1, "@", false);

                temp = (temp - int(temp)) * 60;
                len = floatToString(line, int(temp), 0);
                LCDPrintString(2, pos, line, false);
                pos += len - 1;
                LCDPrintString(2, pos - 1, "'", false);

                temp = (temp - int(temp)) * 60;
                len = floatToString(line, int(temp), 0);
                LCDPrintString(2, pos, line, false);
                pos += len - 1;
                LCDPrintString(2, pos - 1, "\"", true);

                alt = (alt > 9999) ? 999 : alt;
                len = floatToString(line, alt, 0);
                LCDPrintString(3, 0, "Alt: 0000m", false);
                LCDPrintString(3, 11 - len, line, false);

                course = (course > 360) ? 999 : course;
                len = floatToString(line, course, 0);
                LCDPrintString(3, 10, " Crse: 000@", false);
                LCDPrintString(3, 22 - len, line, true);
            #endif /* ENABLE_TINYGPS || ENABLE_SIRFGPS */

            #ifdef ENABLE_FREEIMU
                len, pos = 0;
                LCDPrintString(5, pos, "Yaw: ", false);
                pos += 5;
                len = floatToString(line, ypr[0], 1);
                LCDPrintString(5, pos, line, false);
                pos += len - 1;

                LCDPrintString(5, pos, " Pch: ", false);
                pos += 6;
                len = floatToString(line, ypr[1], 1);
                LCDPrintString(5, pos, line, false);
                pos += len - 1;
                while (pos < 21) {
                    LCDPrintString(5, pos++, " ", false);
                }
                LCDPrintString(5, pos, "\0", true);

                pos = 0;
                LCDPrintString(6, pos, "Roll: ", false);
                pos += 6;
                len = floatToString(line, ypr[2], 2);
                LCDPrintString(6, pos, line, true);
                pos += len;
            #endif /* ENABLE_FREEIMU */

            #ifdef ENABLE_HMC6352
                sprintf(line, "%03d", int(heading));
                LCDPrintString(6, 11, " Hdg: ", false);
                LCDPrintString(6, 17, line, false);
                LCDPrintString(6, 20, "@", true);
            #endif /* ENABLE_HMC6352 */

            #ifdef ENABLE_BMP085
                len = floatToString(line, float(Altitude)/100.0, 0);
                LCDPrintString(4, 0, "Alt: 0000m ", false);
                LCDPrintString(4, 11 - len, line, false);

                len = floatToString(line, float(Temperature)/10.0, 1);
                LCDPrintString(4, 11, "Tmp:", false);
                LCDPrintString(4, 15, line, false);
                LCDPrintString(4, 19, "@C", true);
            #endif /* ENABLE_BMP085 */

            lastTime = millis();
            if (fullRedraw) {
                LCDA.CLEAR();

                sprintf(line, "Photos: %04d   ", photo);
                LCDPrintString(0, 0, line, false);

                LCDA.DrawFullScreen(screenBuffer);
                fullRedraw = false;
            }
        }
    #endif /* ENABLE_LCD12864 */

    usefulDelay(0);

    /* Take a photo */
    if (goprotrigger) {
        /* See if the cameras are ready to take the photo */
        int slavesReady = 0;
        #ifdef ENABLE_LCD12864
            LCDPrintString(1, 0, "Ready to Trig: ", false);
        #endif /* ENABLE_LCD12864 */
        for (int i = 0; i < NUM_SLAVES; i++) {
            bool ready = digitalRead(GOPRO_SLAVES[i]) == LOW;
            slavesReady += (ready) ? 1 : 0;
            #ifdef ENABLE_LCD12864
                if (ready) {
                    LCDPlaceCharacter(1, 15 + i + 1, i + '1');
                } else {
                    LCDPlaceNegCharacter(1, 15 + i + 1, i + '1');
                }
            #endif /* ENABLE_LCD12864 */
        }
        #ifdef ENABLE_LCD12864
            LCDPrintString(1, 15, "\0", true);
        #endif /* ENABLE_LCD12864 */

        if (slavesReady == NUM_SLAVES) {
            /* Trigger camera */
            digitalWrite(GOPRO_TRIG, LOW);
            delay(1);
            digitalWrite(GOPRO_TRIG, HIGH);

            /* Record to SD Card */
            stringToSDCard("P");
            longToSDCard(++photo);

            /* Record to EEPROM */
            eeprom_update_dword((uint32_t *) 0x01, sector);
            eeprom_update_dword((uint32_t *) 0x05, photo);

            /* Update LCD */
            #ifdef ENABLE_LCD12864
                sprintf(line, "Photos: %04d   ", photo);
                LCDPrintString(0, 0, line, true);
            #endif /* ENABLE_LCD12864 */

            #ifdef ENABLE_BEEP
                tone(BEEP_PIN, 2200, 50);
            #endif /* ENABLE_BEEP */

            usefulDelay(300);

            /* Back to normal state */
            digitalWrite(GOPRO_ID2, HIGH);
            digitalWrite(GOPRO_ID3, LOW);
            lastPhoto = millis();
            goprotrigger = false;
            retryCount = 0;

            #ifdef ENABLE_LCD12864
                LCDPrintString(1, 0, "                          ", true);
            #endif /* ENABLE_LCD12864 */
        } else if (millis() - lastPhoto > 5000) {
            /* A camera has taken too long, retry */
            goprotrigger = false;
            retryCount++;

            /* Back to normal state */
            digitalWrite(GOPRO_ID2, HIGH);
            digitalWrite(GOPRO_ID3, LOW);

            if (retryCount < 3) {
                /* Retry again */
                stringToSDCard("R");
                intToSDCard(retryCount);
        
                #ifdef ENABLE_LCD12864
                    LCDPrintString(0, 0, "Camera error", false);
                #endif /* ENABLE_LCD12864 */
                
                #ifdef ENABLE_BEEP
                    tone(BEEP_PIN, 2200, 50);
                    delay(55);
                    tone(BEEP_PIN, 2200, 50);
                    delay(55);
                    tone(BEEP_PIN, 2200, 50);
                    delay(55);
                    tone(BEEP_PIN, 2200, 50);
                    usefulDelay(2000);
                #else
                    usefulDelay(2170);
                #endif /* ENABLE_BEEP */
            } else {
                /* Giving up for good, flush SD card write buffer */
                byte null[1] = { 0 };
                while (bufferPos) {
                    addToSDCard(null, 1);
                }

                /* Remember where we were up to */
                eeprom_update_dword((uint32_t *) 0x01, sector);
                eeprom_update_dword((uint32_t *) 0x05, photo);
                
                /* Clear screen */
                LCDPrintString(1, 0, "                     ", true);
                LCDPrintString(2, 0, "                     ", true);
                LCDPrintString(3, 0, "     CAMERA FAIL     ", true);
                LCDPrintString(4, 0, "                     ", true);
                LCDPrintString(5, 0, "                     ", true);
                LCDPrintString(6, 0, "                     ", true);

                /* Annoying "i'm dead" beep */
                while (true) {
                    #ifdef ENABLE_BEEP
                        tone(BEEP_PIN, 1800, 800);
                        delay(1000);
                    #endif /* ENABLE_BEEP */
                }
            }

            lastPhoto = millis();
        }
    } else if (millis() > lastPhoto + PHOTO_DELAY) {
        /*  Check all cameras are ready */
        #ifdef ENABLE_LCD12864
            LCDPrintString(1, 0, "Cameras ready: ", false);
        #endif /* ENABLE_LCD12864 */

        int slavesReady = 0;
        for (int i = 0; i < NUM_SLAVES; i++) {
            bool ready = digitalRead(GOPRO_SLAVES[i]) == HIGH;
            slavesReady += (ready) ? 1 : 0;
            #ifdef ENABLE_LCD12864
                if (ready) {
                    LCDPlaceCharacter(1, 15 + i + 1, i + '1');
                } else {
                    LCDPlaceNegCharacter(1, 15 + i + 1, i + '1');
                }
            #endif /* ENABLE_LCD12864 */
        }
        #ifdef ENABLE_LCD12864
            LCDPrintString(1, 15, "\0", true);
        #endif /* ENABLE_LCD12864 */

        if (slavesReady == NUM_SLAVES) {
            /* Tell cameras to prepare to take a photo */
            digitalWrite(GOPRO_ID2, LOW);
            digitalWrite(GOPRO_ID3, HIGH);
            goprotrigger = true;
            lastPhoto = millis();
        }
    }
}

/**
 * Delay x ms but do useful things during the delay
 */
void usefulDelay(int ms) {
    unsigned long delayStart = millis();
    do {
        /* Feed TinyGPS with data from GPS */
        #if defined(ENABLE_TINYGPS) || defined(ENABLE_SIRFGPS)
            while (GPS_SERIAL.available()) {
                if (gps.encode(GPS_SERIAL.read())) {
                    newGPSData = true;
                }
            }
        #endif /* ENABLE_TINYGPS || ENABLE_SIRFGPS */

        /* Update IMU data */
        #ifdef ENABLE_FREEIMU
            /* 400kHz I2C on 16Mhz chip */
            my3IMU.getYawPitchRoll(ypr);
        #endif /* ENABLE_FREEIMU */
    } while (millis() - delayStart < ms);
}

/**
 * Write a float to the SD card as 4 bytes
 */
void floatToSDCard(float number)
{
    union u_tag {
        byte c[4];
        float fval;
    } u;

    u.fval = number;
    addToSDCard(u.c, 4);
}

/**
 * Write a long to the SD card as 4 bytes
 */
void longToSDCard(long number)
{
    union u_tag {
        byte c[4];
        long lval;
    } u;

    u.lval = number;
    addToSDCard(u.c, 4);
}

/**
 * Write an int to the SD card as 2 bytes
 */
void intToSDCard(int number)
{
    union u_tag {
        byte c[2];
        int ival;
    } u;

    u.ival = number;
    addToSDCard(u.c, 2);
}

/**
 * Write a string to the SD card
 */
void stringToSDCard(char * data)
{
    byte data2[10];
    int length = 0;
    while (data[length] && length < 10) {
        data2[length] = byte(data[length]);
        length++;
    }
    addToSDCard(data2, length);
}

void addToSDCard(byte * data, int length)
{
    int i = 0;
    while (i < length) {
        buffer[bufferPos++] = data[i];
        i++;

        if (bufferPos == 512) {
            byte sdRetry = 0;
            int error = 1;
            while (sdRetry <= 3) {
                /* Write a full block to card */
                error = SDCARDmodded.writeblock(sector, SS_PIN);
                sdRetry++;
                if (!error) {
                    /* Save was sucessful */
                    LCDPrintString(0, 14, "Y", true);
                    break;
                } else {
                    /* Save failed */
                    #ifdef ENABLE_LCD12864
                        LCDPrintString(0, 0, "SDCard error", false);
                    #endif /* ENABLE_LCD12864 */

                    /* Remember where we were up to */
                    eeprom_update_dword((uint32_t *) 0x01, sector);
                    eeprom_update_dword((uint32_t *) 0x05, photo);
                    
                    #ifdef ENABLE_BEEP
                        tone(BEEP_PIN, 1800, 50);
                        delay(55);
                        tone(BEEP_PIN, 1800, 50);
                        delay(55);
                        tone(BEEP_PIN, 1800, 50);
                    #endif /* ENABLE_BEEP */
                    if (sdRetry == 3) {
                        /* Try the next sector as a last resort */
                        sector++;
                    }
                }
            }
            #ifdef ENABLE_LCD12864
                if (error) {
                    /* Save really failed :-( - data lost */
                    LCDPrintString(0, 14, "N", true);
                }
            #endif /* ENABLE_LCD12864 */

            bufferPos = 0;
            sector++;
        }
    }
}

#ifdef ENABLE_LCD12864
    void LCDClearScreen()
    {
        for (int i = 0; i < 1024; i++) {
            screenBuffer[i] = 0x00;
        }
        LCDA.DrawFullScreen(screenBuffer);
    }

    void LCDPrintString(unsigned int row, unsigned int col, char* string, bool update)
    {
        int i = 0;
        while (string[i]) {
            LCDPlaceCharacter(row, col, string[i]);
            i++;
            col++;
        }

        /* Redraw the row the changed */
        if (update) {
            LCDA.DrawScreenRow(screenBuffer, (row * FONT_HEIGHT) + 1);
        }
    }

    void LCDPlaceCharacter(unsigned int row, unsigned int col, char character)
    {
        if (row > 6 || col >= 21) {
            /* Off the bottom or side */
            return;
        }

        if (fontLookup[byte(character)] == 0 && byte(character) != ' ') {
            /* No character in font */
            return;
        }

        /* What bit in the start character does this start? */
        int startBit = (col * 6) % 8;

        /* Add all 10 character rows */
        for (int i = 0; i < FONT_HEIGHT; i++) {
            /* Pixel are in the lower 5 bits */
            unsigned char pixels = pgm_read_byte_near(fontPixel + (9 * fontLookup[byte(character)]) + i);

            /* Work out what byte this row starts in  */
            int startByte = (i + (row * FONT_HEIGHT) + 1) * 16;
            if (startBit <= 3) {
                /* All within one byte */
                int bits = pixels << 3 - startBit;
                int mask = 0x1F << 3 - startBit;
                screenBuffer[startByte + ((col * 6) / 8)] &= ~mask; // Clear space
                screenBuffer[startByte + ((col * 6) / 8)] |= bits; // Add character
            } else {
                /* Accross two bytes */
                int bits = pixels >> startBit - 3;
                int mask = 0x1F >> startBit - 3;
                screenBuffer[startByte + ((col * 6) / 8)] &= ~mask; // Clear space
                screenBuffer[startByte + ((col * 6) / 8)] |= bits; // Add character

                bits = pixels << (11 - startBit);
                mask = 0x1F << (11 - startBit);
                screenBuffer[startByte + ((col * 6) / 8) + 1] &= ~mask; // Clear space
                screenBuffer[startByte + ((col * 6) / 8) + 1] |= bits; // Add character
            }
        }
    }

    void LCDPlaceNegCharacter(unsigned int row, unsigned int col, char character)
    {
        if (row > 6) {
            /* Off the bottom or side */
            return;
        }

        if (fontLookup[byte(character)] == 0 && character != ' ') {
            /* No character in font */
            return;
        }

        /* What bit in the start character does this start? */
        int startBit = (col * 6) % 8;

        /* Add all 10 character rows */
        for (int i = 0; i < FONT_HEIGHT; i++) {
            /* Pixel are in the lower 5 bits */
            unsigned char pixels = 0x1F & ~pgm_read_byte_near(fontPixel + (9 * fontLookup[byte(character)]) + i);

            /* Work out what byte this row starts in  */
            int startByte = (i + (row * FONT_HEIGHT) + 1) * 16;
            if (startBit <= 3) {
                /* All within one byte */
                int bits = pixels << 3 - startBit;
                int mask = 0x1F << 3 - startBit;
                screenBuffer[startByte + ((col * 6) / 8)] &= ~mask; // Clear space
                screenBuffer[startByte + ((col * 6) / 8)] |= bits; // Add character
            } else {
                /* Accross two bytes */
                int bits = pixels >> startBit - 3;
                int mask = 0x1F >> startBit - 3;
                screenBuffer[startByte + ((col * 6) / 8)] &= ~mask; // Clear space
                screenBuffer[startByte + ((col * 6) / 8)] |= bits; // Add character

                bits = pixels << (11 - startBit);
                mask = 0x1F << (11 - startBit);
                screenBuffer[startByte + ((col * 6) / 8) + 1] &= ~mask; // Clear space
                screenBuffer[startByte + ((col * 6) / 8) + 1] |= bits; // Add character
            }
        }
    }
#endif /* ENABLE_LCD12864 */


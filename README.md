Trackview Camera Controller
===========================

This is the code that runs my trackview camera. I performs 3 main tasks, first it records a bunch of sensor data like GPS, Barometer, Pitch, Roll, Heading.
Second it triggers the GoPro cameras at regular intervals. Third it updates a graphical LCD will sensor values and camera status.

Hardware
--------

* [Seeduino Mega](http://nicegear.co.nz/arduino-boards/seeeduino-mega/) running at 3.3v
* [MicroSD shield](http://www.mindkits.co.nz/store/arduino-compatible/microsd-shield) you'll need to modify this to work on the mega as the SPI pins on the mega are different 
* [BMP085 Barometric Pressure Sensor](http://www.mindkits.co.nz/store/sensors/barometric-pressure-sensor-bmp085-breakout) - optional
* [ITG3200 ADXL345 combo board](http://www.mindkits.co.nz/store/sensors/movement-and-position/imu-digital-combo-board-6-degrees-of-freedom-itg3200-adxl345) - optional
* [HMC5843 Triple Axis Magnetometer Breakout](http://www.mindkits.co.nz/store/sensors/magnetic/triple-axis-magnetometer-breakout-hmc5843) - optional
* [HMC6352 Compass Module](http://www.mindkits.co.nz/store/sensors/compass-module-hmc6352) - optional
* LDR - optional
* Piezo Speaker - optional
* [ST7920 based 128x64 graphical LCD](http://www.mindkits.co.nz/store/led-lcds/128x64-graphic-lcd) - optional
* Any serial GPS (module) outputting NMEA sentences - optional
* 5 x GoPro Hero HD cameras
* 5 x GoPro Slaver boards (see [this thread](http://goprouser.freeforums.org/the-gopro-hero-hd-bus-interface-t797.html) for details)


Software
--------

The code in this repo has a bunch of options at the top. You should be able to comment out and of the `ENABLE_` defines at the top if you're not using that particular piece of hardware.

The code relies on a number of other libraries to hand the sensors. They are:

* [TinyGPS](http://arduiniana.org/libraries/tinygps/)
* [ADXL345](http://code.google.com/p/adxl345driver/source/browse/#svn%2Fbranches%2Ffvaresano)
* [HMC58X3](https://launchpad.net/hmc58x3)
* [ITG3200](http://code.google.com/p/itg-3200driver/source/browse/#svn%2Ftrunk)
* [FreeIMU](http://www.varesano.net/projects/hardware/FreeIMU)
* [hmc6352](http://rubenlaguna.com/wp/2009/03/19/arduino-library-for-hmc6352/index.html)
* [BMP085](http://code.google.com/p/bmp085driver/)
* [SDCARDmodded](http://supertechman.blogspot.com/2011/02/sdcard-library.html)

Download all these and put them in you sketchbook/libraries folder (create this if it doesn't already exist). I may have patched some of these, I will create diffs in the future if that is the case.

The data is written directly to the SD card, from start to end. The format isnt documented yet but the basics are:
    A single character long string followed by a number of float/long/ints.
    eg:
        `"G", 5 x floats, 1 x long` is GPS data (lat, lon, alt, course, speed, age)
        `"I", 3 x floats` is IMU data (pitch, roll, yaw)

In the future I'll create another project with code to process this data. It will rely on being able to read the SD card via `dd`, Microsoft Windows developers will need to create thier own solution.


TODO
----

* Add reset button to reset photo number and SD card position
* Add start button to start taking photos instead of starting imediatly
* Add stop button to stop taking photos and flush SD card buffer
* Flush SD card buffer when retry limit hit
* Alert/retry when SD card writting fails


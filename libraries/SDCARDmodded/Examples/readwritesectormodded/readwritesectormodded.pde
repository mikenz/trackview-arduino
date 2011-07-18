/* 
  Make use of a slightly modded version of the SDCARD lib. This sketch
  is very much the same as the original (thanks Author!).
  
    1. uses arduino SPI lib
    2. allows for choosing SPI SS pin (instead of D10)
    
    Rui Alves supertechman.blogspot.com

   Connect sd card as shown in circuit diagram (original SDCARD lib).
   or quickly read this article (the setup I have):
   http://supertechman.blogspot.com/2011/01/convert-old-ethernet-shield-to-sdcard.html 
   
   Copy SDCARDmodded folder into arduino-0022/hardware/libraries.
   This code will write/read directly to a sector on the sd card
     sector number 0 to 1980000 for a 1.0GB card.
     
   USAGE: download sketch to arduino, open serial Monitor, send an "r".
*/

#include <SDCARDmodded.h>
#include <SPI.h>

unsigned char buffer[512] ;     //this 512 bytes read from or written to sd card
unsigned long sector = 10000;   //the sector we will write or read from - leave lower sectors
int error = 0;                  //the error will be 0 if the sd card works

void setup()			
{
 Serial.begin(9600);            //initialize serial communication with computer
 pinMode(4, OUTPUT);            //This is the SS pin we are going to use 
 pinMode(10, OUTPUT);           //it's a good idea to do this (see SPI lib)
 SPI.begin();                   //the SDCARDmodded lib already sets pin 10 as output
}                               //'but it's never too much ;-)

void loop()			  
{
   if (Serial.available() > 0)   // do nothing if have not received a byte by serial
   {
    int inByte = Serial.read();
    if (inByte == 'r')           // send a "r" to start the read / write sector process
    {
	int i = 0;               //general purpose counter
        for(i=0;i<512;i++)
	buffer[i]=0x15;          //fill the buffer with a number between 0 and 255
	
         unsigned long stopwatch = millis();           //start stopwatch
	 error = SDCARDmodded.writeblock(sector, 4);   //write the buffer to this sector on the sd card
         Serial.print(millis() - stopwatch);
         Serial.println("   ms to write one sector");
	
	 if (error !=0)
     {
         Serial.print("sd card write error... code =  ");
         Serial.println(error);	 
     }                                                  //end of if we have an error
	 
        stopwatch = millis();                           //start stopwatch
	error = SDCARDmodded.readblock(sector, 4);      //read into the buffer this sector in sd card
        
        Serial.print(millis() - stopwatch);
        Serial.println("   ms to read one sector");

         if (error !=0)
     {
         Serial.print("sd card read error... code =  ");
         Serial.println(error);	 
     }                                                  //end of if we have an error
	else
     {
       int number =0;
       for(i=0; i<512; i++)                             //read 512 bytes
     {
       number = buffer[i];                              //convert to integer       
       if(((i % 16) == 0) & ( i != 0))  
       Serial.println("");                              //write 16 then start a new line
       Serial.print(number);
       Serial.print("  ");                              //tab to next number
     }                                                  //end of read 512 bytes
     Serial.println(" ");
     }                                                  //end of else we have no error
    }                                                   //end of received "r"
   }                                                    //end of serial available
}                                                       //end of loop
 




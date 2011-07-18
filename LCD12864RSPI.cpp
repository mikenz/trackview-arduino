//Demo LCD12864 spi
//www.dfrobot.com


#include "LCD12864RSPI.h"


extern "C" 
{
#include <wiring.h> 
#include <inttypes.h>
#include <stdio.h>  //not needed yet
#include <string.h> //needed for strlen()
#include <avr/pgmspace.h>
}



LCD12864RSPI::LCD12864RSPI() 
{
this->DEFAULTTIME = 1; // 80 ms default time
this->delaytime = DEFAULTTIME;
} 

//*********************ʱ************************//
void LCD12864RSPI::delayns(void)
{   
delayMicroseconds(delaytime);
 }


void LCD12864RSPI::WriteByte(int dat)
{
    digitalWrite(latchPin, HIGH);
    delayns();
    shiftOut(dataPin, clockPin, MSBFIRST, dat);
    digitalWrite(latchPin, LOW);
}


void LCD12864RSPI::WriteCommand(int CMD)
{
   int H_data,L_data;
   H_data = CMD;
   H_data &= 0xf0;           //4
   L_data = CMD;             //xxxx0000ʽ
   L_data &= 0x0f;           //θ4
   L_data <<= 4;             //xxxx0000ʽ
   WriteByte(0xf8);          //RS=0д
   WriteByte(H_data);
   WriteByte(L_data);
}


void LCD12864RSPI::WriteData(int CMD)
{
   int H_data,L_data;
   H_data = CMD;
   H_data &= 0xf0;           //4
   L_data = CMD;             //xxxx0000ʽ
   L_data &= 0x0f;           //θ4
   L_data <<= 4;             //xxxx0000ʽ
   WriteByte(0xfa);          //RS=1д
   WriteByte(H_data);
   WriteByte(L_data);
}



void LCD12864RSPI::Initialise()
{
    pinMode(latchPin, OUTPUT);     
    pinMode(clockPin, OUTPUT);    
    pinMode(dataPin, OUTPUT);
    digitalWrite(latchPin, LOW);
    delayns();

    WriteCommand(0x30);        //趨
    WriteCommand(0x0c);        //ʾؿ
    WriteCommand(0x01);        //Ļ
    WriteCommand(0x06);        //趨
}


void LCD12864RSPI::CLEAR(void)
{  
    WriteCommand(0x30);//
    WriteCommand(0x01);//ʾ
}


void LCD12864RSPI::DisplayString(int X,int Y, char *ptr,int dat)
{
  int i;

  switch(X)
   {
     case 0:  Y|=0x80;break;

     case 1:  Y|=0x90;break;

     case 2:  Y|=0x88;break;

     case 3:  Y|=0x98;break;

     default: break;
   }

  WriteCommand(Y); // ʾʼַ

  for(i=0;i<dat;i++)
    { 
      WriteData((unsigned int)ptr[i]);//ʾʱעֵʾһ
    }
}



void LCD12864RSPI::DisplaySig(int M,int N,int sig)
{
  switch(M)
   {
     case 0:  N|=0x80;break;

     case 1:  N|=0x90;break;

     case 2:  N|=0x88;break;

     case 3:  N|=0x98;break;

     default: break;
   }
  WriteCommand(N); // ʾʼַ
  WriteData(sig); //ַ
 }




void LCD12864RSPI::DrawFullScreen(uchar *p)
{
    int row, x, y, i;
    int temp;
    int tmp;
     
    for (row = 0; row < 64; row++) {                           //д
        if (row < 32) {
            x = 0x80;
            y = row+0x80;
        } else {
            x = 0x88;
            y = row - 32 + 0x80;    
        }         
        WriteCommand(0x34);
        WriteCommand(y);
        WriteCommand(x);
        WriteCommand(0x30);
        tmp = row * 16;
        for(i = 0; i < 16; i++) {
		    temp = p[tmp++];
		    WriteData(temp);
        }
    }
    WriteCommand(0x34);
    WriteCommand(0x36);
}

void LCD12864RSPI::DrawScreenRow(uchar *p, int rowStart = 0)
{
    int row, x, y;
    int temp;
    int tmp;
     
    for (row = rowStart; row < rowStart + 9; row++) {
        if (row < 32) {
            x = 0x80;
            y = row + 0x80;
        } else {
            x = 0x88;
            y = row - 32 + 0x80;    
        }         
        
        WriteCommand(y);
        WriteCommand(x);

        tmp = row * 16;
        for (int i = 0; i < 16; i++) {
		    temp = p[tmp++];
		    WriteData(temp);
        }
    }
}

LCD12864RSPI LCDA = LCD12864RSPI();

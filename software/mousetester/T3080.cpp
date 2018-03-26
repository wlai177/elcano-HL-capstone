#include"Arduino.h"
#include"T3080.h"
#include "SPI.h"
void T3080::init_1()
{
pinMode(ADNS3080_CHIP_SELECT,OUTPUT);
pinMode(ADNS3080_POWER_DOWN,OUTPUT);
pinMode(ADNS3080_RESET,OUTPUT);
digitalWrite(ADNS3080_CHIP_SELECT,LOW);
digitalWrite(ADNS3080_RESET, LOW);
digitalWrite(ADNS3080_POWER_DOWN,HIGH);
}
void T3080::init_2()
{
int retry = 0;
byte productId = 0;
byte revisionId = 0;
// PRODUCT ID VERIFICATION
while( retry < 10 ) {
delayMicroseconds(75);
// take the chip select low to select the device
digitalWrite(ADNS3080_CHIP_SELECT,LOW);
// send the device the register you want to read:
SPI.transfer(ADNS3080_PRODUCT_ID);
// small delay
delayMicroseconds(75);
// send a value of 0 to read the first byte returned:
//productId = SPI.transfer(0x00);
if( productId == 0x00 ) {
Serial.println("\n Found productId ");
Serial.print(productId, HEX);;
}
else{
Serial.println("\n False productId ");
Serial.print(productId, HEX);;
}
retry++;
}
/*
if(productId != 0x17) {
delay(100);
exit(1);
}
*/
// REVISION ID VERIFICATION
delayMicroseconds(75);
// take the chip select low to select the device
digitalWrite(ADNS3080_CHIP_SELECT,LOW);
// send the device the register you want to read:
SPI.transfer(ADNS3080_REVISION_ID);
// small delay
delayMicroseconds(75);
// send a value of 0 to read the first byte returned:
revisionId = SPI.transfer(0x00);
Serial.println("\n rev");
Serial.print(revisionId, HEX);
// Set resolution to 1600 counts/inch
delayMicroseconds(75);
// set the chip select to low to select the device
digitalWrite(ADNS3080_CHIP_SELECT, LOW);
// send register address
SPI.transfer(ADNS3080_CONFIGURATION_BITS | 0x80 );
// send data
SPI.transfer(0x10);
}
byte T3080::read_register(byte address)
{
byte data = 0;
delayMicroseconds(75);
// take the chip select low to select the device
digitalWrite(ADNS3080_CHIP_SELECT,LOW);
// send the device the register you want to read:
SPI.transfer(address);
// small delay
delayMicroseconds(75);
// send a value of 0 to read the first byte returned:
data = SPI.transfer(0x00);
// take the chip select high to de-select:
digitalWrite(ADNS3080_CHIP_SELECT, HIGH);
return data;
}
void T3080::write_register(byte address, byte data)
{
delayMicroseconds(75);
// set the chip select to low to select the device
digitalWrite(ADNS3080_CHIP_SELECT, LOW);
// send register address
SPI.transfer(address | 0x80 );
// send data
SPI.transfer(data);
// set the chip select to high to de-select the device
digitalWrite(ADNS3080_CHIP_SELECT, HIGH);
}
void T3080::measurement()
{
byte motion = 0;
uint8_t dx = 0;
uint8_t dy = 0;
int8_t delta_x = 0;
int8_t delta_y = 0;
float DELTA_x = 0;
float DELTA_y = 0;
byte SQUAL = 0;
delayMicroseconds(75);
// take the chip select low to select the device
digitalWrite(ADNS3080_CHIP_SELECT,LOW);
// send the device the register you want to read:
SPI.transfer(ADNS3080_MOTION);
// small delay
delayMicroseconds(75);
// send a value of 0 to read the first byte returned:
motion = SPI.transfer(0x00);
delayMicroseconds(75);
// send the device the register you want to read:
SPI.transfer(ADNS3080_DELTA_X);
// small delay
delayMicroseconds(75);
// send a value of 0 to read the first byte returned:
dx = SPI.transfer(0x00);
delayMicroseconds(75);
// send the device the register you want to read:
SPI.transfer(ADNS3080_DELTA_Y);
// small delay
delayMicroseconds(75);
// send a value of 0 to read the first byte returned:
dy = SPI.transfer(0x00);
delayMicroseconds(75);
// send the device the register you want to read:
SPI.transfer(ADNS3080_SQUAL);
// small delay
delayMicroseconds(75);
// send a value of 0 to read the first byte returned:
SQUAL = SPI.transfer(0x00);
// check for overflow
if( (motion & 0x10) != 0 )
{
Serial.println("\n Attention Overflow");
}
else
{
Serial.println("\n No Overflow");
}
// check resolution
if( (motion & 0x01) != 0 )
{
Serial.println("\n Resolution = 1600 counts/inch");
}
else
{
Serial.println("\n Resolution = 400 counts/inch");
}
// check for motion and update dx and dy
if( (motion & 0x80) != 0 )
{
Serial.println("\n Motion");
}
else
{
Serial.println("\n No Motion");
}
delta_x= (int8_t)dx;
delta_y= (int8_t)dy;
DELTA_x=(float)delta_x/1600.0;
DELTA_y=(float)delta_y/1600.0;
DELTA_x=DELTA_x/1.62914206;
DELTA_y=DELTA_y/1.62914206;
DELTA_x=100*DELTA_x;
DELTA_y=100*DELTA_y;
x_pos=x_pos+DELTA_x; // x position in cm
y_pos=y_pos+DELTA_y; // y position in cm
//Display DELTA
Serial.print(DELTA_x, DEC);
Serial.print(" ");
Serial.print(DELTA_y, DEC);
//Display x and y
Serial.print(" ");
Serial.print(x_pos, DEC);
Serial.print(" ");
Serial.println(y_pos, DEC);
//Display Quality
Serial.println("\n Quality");
Serial.print(SQUAL, DEC);
// take the chip select high to de-select:
// digitalWrite(ADNS3080_CHIP_SELECT, HIGH);
// delayMicroseconds(5);
}
void T3080::reset()
{
digitalWrite(ADNS3080_RESET,HIGH); // reset sensor
delayMicroseconds(10);
digitalWrite(ADNS3080_RESET,LOW); // return sensor to normal
}

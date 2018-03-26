#ifndef T3080_h
#define T3080_h
#include "Arduino.h"
// Register Map for the ADNS3080 Optical Optical Flow Sensor
#define ADNS3080_PRODUCT_ID 0x00
#define ADNS3080_REVISION_ID 0x01
#define ADNS3080_MOTION 0x02
#define ADNS3080_DELTA_X 0x03
#define ADNS3080_DELTA_Y 0x04
#define ADNS3080_SQUAL 0x05
#define ADNS3080_PIXEL_SUM 0x06
#define ADNS3080_MAXIMUM_PIXEL 0x07
#define ADNS3080_CONFIGURATION_BITS 0x0a
#define ADNS3080_EXTENDED_CONFIG 0x0b
#define ADNS3080_DATA_OUT_LOWER 0x0c
#define ADNS3080_DATA_OUT_UPPER 0x0d
#define ADNS3080_SHUTTER_LOWER 0x0e
#define ADNS3080_SHUTTER_UPPER 0x0f
#define ADNS3080_FRAME_PERIOD_LOWER 0x10
#define ADNS3080_FRAME_PERIOD_UPPER 0x11
#define ADNS3080_MOTION_CLEAR 0x12
#define ADNS3080_FRAME_CAPTURE 0x13
#define ADNS3080_SROM_ENABLE 0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER 0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER 0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER 0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER 0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER 0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER 0x1e
#define ADNS3080_SROM_ID 0x1f
#define ADNS3080_OBSERVATION 0x3d
#define ADNS3080_INVERSE_PRODUCT_ID 0x3f
#define ADNS3080_PIXEL_BURST 0x40
#define ADNS3080_MOTION_BURST 0x50
#define ADNS3080_SROM_LOAD 0x60
#define FRAME_LENGTH 900
class T3080
{
public:
T3080() {
ADNS3080_CHIP_SELECT = 4; // chip select pin
ADNS3080_RESET = 5; // chip reset pin
ADNS3080_POWER_DOWN = 6; // Power down pin
x_pos = 0;
y_pos = 0;
}
void measurement(); // sends values of dx and dy over the serial link and increments distx and disty respectively by dx and dy
void reset(); // reset sensor
byte read_register(byte address);
void init_1();
void init_2();
void write_register(byte address, byte data);
private:
int ADNS3080_CHIP_SELECT; // chip select pin
int ADNS3080_RESET;
int ADNS3080_POWER_DOWN;
float x_pos;
float y_pos;
};
#endif
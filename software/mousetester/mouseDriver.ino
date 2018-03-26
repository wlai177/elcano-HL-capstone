#include "T3080.h"
#include <SPI.h>
T3080 sensor;
typedef unsigned long Time;
void setup() {
sensor.init_1();
Serial.begin(115200);
Serial.println();
SPI.begin();
SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE3);
SPI.setClockDivider(SPI_CLOCK_DIV16);
sensor.reset();
sensor.init_2();
}
void loop() {
static Time last_reset;
Time now = millis();
sensor.measurement();
delay(500);
}


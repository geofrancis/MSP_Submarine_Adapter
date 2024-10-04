#include <Wire.h>
#include <MSP.h>
#include "MS5611.h"
#include "ms4525do.h"


bfs::Ms4525do pres;

byte received;

float diffPressurePa;
int16_t temp;


int distances = 0;
int distance = 0;

unsigned char buffer_RTT[4] = { 0 };
uint8_t CS;
#define COM 0x55
int Distance = 0;

MSP msp;

MS5611 MS5611(0x77);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  msp.begin(Serial2);
  Wire.begin(22, 23);  //  adjust ESP32 pins if needed
  pres.Config(&Wire, 0x28, 1.0f, -1.0f);
}

void loop() {

  MS5611.read();
  Serial.print("T:\t");
  Serial.print(MS5611.getTemperature(), 2);
  Serial.print("\tP:\t");
  Serial.print(MS5611.getPressure(), 2);
  Serial.println();





  Serial1.write(COM);
  if (Serial1.available() > 0) {
    delay(1000);
    if (Serial1.read() == 0xff) {
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = Serial1.read();
      }
      CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == CS) {
        distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
        distances = (distance / 10);
      }
    }
  }


 if (pres.Read()) {
    Serial.print(pres.pres_pa(), 6);
    Serial.print("\t");
    Serial.print(pres.die_temp_c(), 6);
    Serial.print("\n");
 }

  uint8_t sonarrange[] = { 255, 130 };                                                        //quality,range,0,0,0
  uint8_t barorange[] = { 1, millis(), MS5611.getPressure(), (MS5611.getTemperature(), 2) };  //quality,range,0,0,0
  uint8_t speed[] = { 1, millis(), pres.pres_pa(), pres.die_temp_c() };


  msp.send(0x1F01, &sonarrange, sizeof(sonarrange));
  msp.send(0x1F05, &barorange, sizeof(barorange));
  msp.send(0x1F06, &speed, sizeof(speed));
}

#include <Wire.h>
#include <MSP.h>
#include "Adafruit_MPRLS.h"
#include "ms4525do.h"
#include <HardwareSerial.h>




MSP msp;


#define RESET_PIN -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN -1    // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

bfs::Ms4525do pres;

unsigned char buffer_RTT[4] = { 0 };
uint8_t CS;
#define COM 0x55
int Distance = 0;

byte received;

float diffPressurePa;
int16_t temp;


int distances = 0;
int distance = 0;

float pressure_hPa = 0;



void setup() {
  Serial.begin(115200);
  Wire.begin(1, 2);  


  delay(3000);
  Serial1.begin(115200, SERIAL_8N1, 3, 4);
  Serial.print("Sonar Serial");

  Serial2.begin(115200, SERIAL_8N1, 5, 6);
  Serial.print("Serial2");
  msp.begin(Serial2);

  Serial.println("MPRLS Simple Test");
  if (!mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
     pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  }
}




void loop() {

  sonar();
  pressure();
  speed();
  MSPsend();

}



void sonar() {
  //sonar
  Serial2.write(COM);
  if (Serial2.available() > 0) {
    if (Serial2.read() == 0xff) {
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = Serial2.read();
      }
      CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == CS) {
        distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
        distances = (distance / 10);
        Serial.print("Serial2 ");
        Serial.println(distance);
        uint8_t sonarrange[] = { 255, distances };
       
      }
    }
  }
}


void pressure() {
  //mprls
  pressure_hPa = mpr.readPressure();
 // Serial.print("Pressure (hPa): ");
 // Serial.println(pressure_hPa);
  //Serial.print("Pressure (PSI): ");
  //Serial.println(pressure_hPa / 68.947572932);
}

void speed() {

  if (pres.Read()) {
    Serial.print(pres.pres_pa(), 6);
    Serial.print("\t");
    Serial.print(pres.die_temp_c(), 6);
    Serial.print("\n");
  }
}

void MSPsend(){


  uint8_t sonarrange[] = { 255, distances };
  uint8_t barorange[] = { 1, millis(), pressure_hPa, (0) };
  uint8_t speed[] = { 1, millis(), pres.pres_pa(), pres.die_temp_c() };

  msp.send(0x1F01, &sonarrange, sizeof(sonarrange));
  msp.send(0x1F05, &barorange, sizeof(barorange));
  msp.send(0x1F06, &speed, sizeof(speed));
  //Serial.println("loop");
  delay(100);

}

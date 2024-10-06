#include <Wire.h>
#include <MSP.h>
//#include "MS5611.h"
#include "Adafruit_MPRLS.h"
#include "ms4525do.h"
#include <HardwareSerial.h>

//Define two Serial devices mapped to the two internal UARTs
HardwareSerial SONAR(0);
HardwareSerial MSPSERIAL(1);


MSP msp;

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN -1    // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

bfs::Ms4525do pres;
//MS5611 MS5611(0x77);

unsigned char buffer_RTT[4] = { 0 };
uint8_t CS;
#define COM 0x55
int Distance = 0;

byte received;

float diffPressurePa;
int16_t temp;


int distances = 0;
int distance = 0;





void setup() {
  Serial.begin(115200);
  Wire.begin(6, 7);  //  adjust ESP32 pins if needed


  delay(3000);
  SONAR.begin(115200, SERIAL_8N1, 1, 0);
  Serial.print("Sonar Serial");

  // And configure MySerial1 on pins RX=D9, TX=D10
  MSPSERIAL.begin(115200, SERIAL_8N1, 20, 21);
  Serial.print("MSPSERIAL");
  msp.begin(MSPSERIAL);

  Serial.println("MPRLS Simple Test");
  if (!mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");

    // pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  }
}




  void loop() {


    SONAR.write(COM);
    if (SONAR.available() > 0) {
      if (SONAR.read() == 0xff) {
        buffer_RTT[0] = 0xff;
        for (int i = 1; i < 4; i++) {
          buffer_RTT[i] = SONAR.read();
        }
        CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
        if (buffer_RTT[3] == CS) {
          distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
          distances = (distance / 10);
          Serial.print("SONAR ");
          Serial.println(distance);
          uint8_t sonarrange[] = { 255, distances };
          msp.send(0x1F01, &sonarrange, sizeof(sonarrange));
        }
      }
    }



    /* MS5611.read();
  Serial.print("T:\t");
  Serial.print(MS5611.getTemperature(), 2);
  Serial.print("\tP:\t");
  Serial.print(MS5611.getPressure(), 2);
  Serial.println();
*/

    /*

  if (pres.Read()) {
    Serial.print(pres.pres_pa(), 6);
    Serial.print("\t");
    Serial.print(pres.die_temp_c(), 6);
    Serial.print("\n");
  }
*/
    uint8_t sonarrange[] = { 255, distances };
    uint8_t barorange[] = { 1, millis(), mpr.readPressure(), (0) };
    //uint8_t barorange[] = { 1, millis(), MS5611.getPressure(), (MS5611.getTemperature()) };
    // uint8_t speed[] = { 1, millis(), pres.pres_pa(), pres.die_temp_c() };

    // int8_t sonarrange[] = { 255, 130 };
    // uint8_t barorange[] = { 1, millis(), 1000, (69, 2) };
    // uint8_t speed[] = { 1, millis(), 67, 42 };

    msp.send(0x1F01, &sonarrange, sizeof(sonarrange));
    msp.send(0x1F05, &barorange, sizeof(barorange));
    // msp.send(0x1F06, &speed, sizeof(speed));
    Serial.println("loop");
    delay(100);
  }

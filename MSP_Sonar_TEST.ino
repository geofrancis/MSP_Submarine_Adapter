


void setup() {
Serial.begin(115200);
Serial1.begin(115200);
}






void loop() {
// Define payload arrays
uint8_t payload1[] = {255, 130, 0, 0, 0};
uint8_t payload2[] = {255, 143, 0, 0, 0};

// Send MSP V2 message for SENSOR_RANGEFINDER
sendMSPMessage(0x1F01, payload1, sizeof(payload1));

// Send MSP V2 message for SENSOR_OPTIC_FLOW
sendMSPMessage(0x1F02, payload2, sizeof(payload2));

delay(1000); // Delay 1 second before sending the next message
}

void sendMSPMessage(uint16_t function, uint8_t payload[], uint16_t payloadSize) {
// Start MSP V2 message with header
Serial1.write('$');
Serial1.write('X');
Serial1.write('<');

// Send function code
Serial1.write((uint8_t)function);
Serial1.write((uint8_t)(function >> 8));

// Send payload size
Serial1.write((uint8_t)payloadSize);
Serial1.write((uint8_t)(payloadSize >> 8));

// Send payload
for (uint16_t i = 0; i < payloadSize; i++) {
Serial1.write(payload[i]);
}

// Calculate and send checksum
uint8_t checksum = 0;
checksum ^= '<';
checksum ^= (uint8_t)function;
checksum ^= (uint8_t)(function >> 8);
checksum ^= (uint8_t)payloadSize;
checksum ^= (uint8_t)(payloadSize >> 8);
for (uint16_t i = 0; i < payloadSize; i++) {
checksum ^= payload[i];
}
Serial1.write(checksum);

// Print the sent message to serial monitor
Serial.print("Sent MSP V2 Message: < Function: 0x");
Serial.print(function, HEX);
Serial.print(" Payload : ");
for (uint16_t i = 0; i < payloadSize; i++) {
Serial.print(payload[i]);
Serial.print(" ");
}
Serial.print(" Payload Size: ");
Serial.print(payloadSize);
Serial.println(" bytes");
}
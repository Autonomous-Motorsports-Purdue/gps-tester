// Transparent UART bridge: GPS (USART1) <-> PC (ST-Link virtual COM port, USART2).
//
// The F303K8 has no native USB, so "USB" here is the ST-Link VCP (/dev/ttyACM*).
// Wiring (Nucleo-32 pin labels):
//   GPS TX -> D0 / PA10 (USART1_RX)
//   GPS RX -> D1 / PA9  (USART1_TX)
//   GND    -> GND
// Data flows both ways, so the PC can also send config commands / RTCM corrections to the GPS.
#include <Arduino.h>

#ifndef GPS_BAUD
#define GPS_BAUD 230400  // measured on this GEM1305 (Unicore default is 115200)
#endif
#define PC_BAUD  460800  // faster than GPS so the PC link never bottlenecks

HardwareSerial GpsSerial(PA10, PA9);  // RX, TX -> USART1

static uint32_t lastActivity = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(PC_BAUD);
  GpsSerial.begin(GPS_BAUD);
}

void loop() {
  uint8_t buf[64];
  size_t n;

  // Only move as many bytes as the destination TX buffer can take, so a burst in
  // one direction (e.g. RTCM corrections from the PC) never blocks the other.
  if ((n = min(min((size_t)GpsSerial.available(), (size_t)Serial.availableForWrite()), sizeof(buf))) > 0) {
    n = GpsSerial.readBytes(buf, n);
    Serial.write(buf, n);
    lastActivity = millis();
  }
  if ((n = min(min((size_t)Serial.available(), (size_t)GpsSerial.availableForWrite()), sizeof(buf))) > 0) {
    n = Serial.readBytes(buf, n);
    GpsSerial.write(buf, n);
  }

  // LED on while GPS data is arriving
  digitalWrite(LED_BUILTIN, millis() - lastActivity < 50 ? HIGH : LOW);
}

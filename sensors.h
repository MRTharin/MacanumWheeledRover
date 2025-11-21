/* Sensor functions for Arduino-based robots */

#ifndef SENSORS_H
#define SENSORS_H

/* Ping sensor - returns distance in cm */
long Ping(int pin) {
  long duration, distance;
  
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
  
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH, 30000);  // 30ms timeout
  
  distance = duration / 29 / 2;
  
  return distance;
}

#endif
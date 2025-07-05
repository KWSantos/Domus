#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#define TRIG 4
#define ECHO 5

void setupUltrasonic() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

long readDistanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH);
  return duration * 0.034 / 2;
}

#endif

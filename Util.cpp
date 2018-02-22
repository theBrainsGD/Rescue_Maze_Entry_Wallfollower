#include "Util.h"

int readLeftFront(SharpIR sensor) {
  int sum = 0;
  int n = 10;
  for (int i = 0; i < n; i++) {
    sum += sensor.distance();
  }

  sum /= n;
  
  return sum;
}


boolean testForVictim() {
  int sensorValue;
  int tempSensorValueRed;
  int tempSensorValueGreen;

  sensorValue = analogRead(LightSensor);
  Serial.println(sensorValue);

  digitalWrite(LEDRed, LOW);
  digitalWrite(LEDGreen, HIGH);
  sensorValue = 0;
  int i = 10;
  int i2 = i;
  for (; i > 0; i--) {
    sensorValue += analogRead(LightSensor);
  }
  tempSensorValueRed = sensorValue / i2;
  Serial.print("Red: ");
  Serial.println(tempSensorValueRed);
  delay(200);

  digitalWrite(LEDRed, HIGH);
  digitalWrite(LEDGreen, LOW);
  sensorValue = 0;
  int j = 10;
  int j2 = j;
  for (; j > 0; j--) {
    sensorValue += analogRead(LightSensor);
  }
  tempSensorValueGreen = sensorValue / j2;
  Serial.print("Green: ");
  Serial.println(tempSensorValueGreen);
  delay(200);

  digitalWrite(LEDRed, HIGH);
  digitalWrite(LEDGreen, HIGH);

  return (diff(tempSensorValueRed, tempSensorValueGreen) >= 0);

}


int diff(int compared, int comparing) {
  return compared - comparing;
}

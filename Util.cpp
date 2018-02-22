#include "Util.h"

int readLeftFront(SharpIR sensor) {
  int sum = 0;
  int n = 10;
  for (int i = 0; i < n; i++) {
    sum += sensor.distance();
  }

  sum /= n;
  //sum /= 2;

  return sum;
}

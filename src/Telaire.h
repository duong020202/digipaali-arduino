#ifndef __Telaire_h
#define __Telaire_h
#endif

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

class T9602 {
  public:
    float temperature=0.0;  float humidity=0.0;
    void getdata(byte *a, byte *b, byte *c, byte *d, byte id);
    void showthedata(byte id);
};

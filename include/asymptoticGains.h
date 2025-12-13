#pragma once

#include <string>
#include <cmath>

class AsymptoticGains {


  private:

    float i, f, k, p, setpoint;


  public:

    AsymptoticGains(float i, float f, float k, float p);

    void setGain(float setpoint);
    float getGain();
};
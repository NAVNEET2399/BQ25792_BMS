#include <Wire.h>
#include "BQ25792.h"

// Creating custom I2C bus 
#define SDA_BMS 33
#define SCL_BMS 32
TwoWire I2CBMS = TwoWire(0);
BQ25792 battery_charging(I2CBMS);

void setup() 
{
  Serial.begin(115200);
  I2CBMS.begin(SDA_BMS, SCL_BMS);
  battery_charging.begin();
}

void loop() 
{
  battery_charging.properties();
  Serial.print("VBUS : "); Serial.println(battery_charging.getVBUS());
  Serial.print("VSYS : "); Serial.println(battery_charging.getVSYS());
  Serial.print("VBAT : "); Serial.println(battery_charging.getVBAT());
  delay(1000);
}

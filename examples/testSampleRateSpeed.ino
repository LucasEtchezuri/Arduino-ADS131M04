#include <Arduino.h>
#include "ADS131M04.h"
#include <SPI.h>

ADS131M04 adc;

void setup()
{
  Serial.begin(115200);
  adc.begin(14, 12, 13, 5, 19);

  delay(1000);
  Serial.println("");

  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setOsr(OSR_128);      // 32KSPS  only with 8MHZ clock
}

void loop()
{
  adcOutput res;
  delay(100);
  unsigned long timeAnt = 0;
  unsigned long cont = 0;

  while (1)
  {
    if (adc.isDataReady())
    {
      res = adc.readADC();
      cont++;
    }
    if (millis() - timeAnt > 1000)
    {
      Serial.print("SPS = ");
      Serial.println(cont);
      timeAnt = millis();
      cont = 0;
    }
  }
}

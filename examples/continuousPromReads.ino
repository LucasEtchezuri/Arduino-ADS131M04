#include <Arduino.h>
#include "ADS131M04.h"

ADS131M04 adc;

void setup()
{
  Serial.begin(115200);
  adc.begin(14, 12, 13, 5, 19);

  delay(1000);
  Serial.println("");

  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_INPUT_SHORTED);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_INPUT_SHORTED);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_INPUT_SHORTED);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_INPUT_SHORTED);
}

void loop()
{
  adcOutput res;
  long ch0 = 0;
  unsigned int cont = 0;
  int prom = 10;

  delay(100);
  while (1)
  {
    while (cont <= prom)
    {
      if (adc.isDataReady())
      {
        res = adc.readADC();
        ch0 = ch0 + res.ch0;
        cont++;
        delay(1);
      }
    }
    Serial.print("Prom Value = ");
    Serial.println(ch0 / prom);
    Serial.println(" ");
    ch0 = 0;
    cont = 0;
    delay(1000);
  }
}

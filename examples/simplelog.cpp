
#include <Arduino.h>

// ADS131M02
#define ADC_MOSI 13
#define ADC_MISO 26
#define ADC_CLK 14
#define ADC_DRDY 39
#define ADC_CS 25
#define ADC_RESET 16

#include "ADS131M0x.h"

SPIClass SpiADC(HSPI);
ADS131M0x adc;

void adcInit()
{
    adc.setClockSpeed(200000);
    adc.reset(ADC_RESET);
    adc.begin(&SpiADC, ADC_CLK, ADC_MISO, ADC_MOSI, ADC_CS, ADC_DRDY);
    adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
    adc.setChannelPGA(0, CHANNEL_PGA_1);
}
void setup()
{
    Serial.begin(9600);
    delay(5);
    Serial.println("INIT");
    adcInit();
    Serial.println("adcInit finished");
}

void loop()
{
    if (adc.isDataReady())
    {
            int32_t Val = adc.readADC().ch0;
            // int32_t Val = adc.readfastCh0();
            Serial.printf("ADC-Val :%d", Val);

            uint32_t valabs = abs(Val);
            uint32_t cnt = valabs / 50000;
            if (Val >= 0)
            {
                for (size_t i = 0; i < cnt; i++)
                {
                    Serial.print('+');
                }
            }
            else
            {
                for (size_t i = 0; i < cnt; i++)
                {
                    Serial.print('-');
                }
            }
            Serial.println();
    }
}

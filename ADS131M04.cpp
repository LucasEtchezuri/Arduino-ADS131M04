#include "Arduino.h"
#include "ADS131M04.h"
#include "SPI.h"

#define settings SPISettings(4000000, MSBFIRST, SPI_MODE1)

ADS131M04::ADS131M04()
{
}

uint8_t ADS131M04::writeRegister(uint8_t address, uint16_t value)
{
  uint16_t res;
  uint8_t addressRcv;
  uint8_t bytesRcv;
  uint16_t cmd = 0;

  digitalWrite(ADS131M04_CS_PIN, LOW);
  delayMicroseconds(1);

  cmd = (CMD_WRITE_REG) | (address << 7) | 0;

  //res = SPI.transfer16(cmd);
  SPI.transfer16(cmd);
  SPI.transfer(0x00);

  SPI.transfer16(value);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  res = SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(ADS131M04_CS_PIN, HIGH);

  addressRcv = (res & REGMASK_CMD_READ_REG_ADDRESS) >> 7;
  bytesRcv = (res & REGMASK_CMD_READ_REG_BYTES);

  if (addressRcv == address)
  {
    return bytesRcv + 1;
  }
  return 0;
}

void ADS131M04::writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask)
{
  // Escribe un valor en el registro, aplicando la mascara para tocar unicamente los bits necesarios.
  // No realiza el corrimiento de bits (shift), hay que pasarle ya el valor corrido a la posicion correcta

  // Leo el contenido actual del registro
  uint16_t register_contents = readRegister(address);

  // Cambio bit aa bit la mascara (queda 1 en los bits que no hay que tocar y 0 en los bits a modificar)
  // Se realiza un AND co el contenido actual del registro.  Quedan "0" en la parte a modificar
  register_contents = register_contents & ~mask;

  // se realiza un OR con el valor a cargar en el registro.  Ojo, valor debe estar en el posicion (shitf) correcta
  register_contents = register_contents | value;

  // Escribo nuevamente el registro
  writeRegister(address, register_contents);
}

uint16_t ADS131M04::readRegister(uint8_t address)
{
  uint16_t cmd;
  uint16_t data;

  cmd = CMD_READ_REG | (address << 7 | 0);

  digitalWrite(ADS131M04_CS_PIN, LOW);
  delayMicroseconds(1);

  //data = SPI.transfer16(cmd);
  SPI.transfer16(cmd);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  data = SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(ADS131M04_CS_PIN, HIGH);
  return data;
}

void ADS131M04::begin(uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin, uint8_t drdy_pin)
{
  // Set pins up
  ADS131M04_CS_PIN = cs_pin;
  ADS131M04_DRDY_PIN = drdy_pin;
  ADS131M04_CLK_PIN = clk_pin;
  ADS131M04_MISO_PIN = miso_pin;
  ADS131M04_MOSI_PIN = mosi_pin;

  SPI.begin(ADS131M04_CLK_PIN, ADS131M04_MISO_PIN, ADS131M04_MOSI_PIN);
  SPI.beginTransaction(settings);
  // Configure chip select as an output
  pinMode(ADS131M04_CS_PIN, OUTPUT);
  // Configure DRDY as as input
  pinMode(ADS131M04_DRDY_PIN, INPUT);
}

int8_t ADS131M04::isDataReadySoft(byte channel)
{
  if (channel == 0)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY0);
  }
  else if (channel == 1)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY1);
  }
  else if (channel == 2)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY2);
  }
  else if (channel == 3)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY3);
  }
  else
  {
    return -1;
  }
}

bool ADS131M04::isResetStatus(void)
{
  return (readRegister(REG_STATUS) & REGMASK_STATUS_RESET);
}

bool ADS131M04::isLockSPI(void)
{
  return (readRegister(REG_STATUS) & REGMASK_STATUS_LOCK);
}

bool ADS131M04::setDrdyFormat(uint8_t drdyFormat)
{
  if (drdyFormat > 1)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_MODE, drdyFormat, REGMASK_MODE_DRDY_FMT);
    return true;
  }
}

bool ADS131M04::setDrdyStateWhenUnavailable(uint8_t drdyState)
{
  if (drdyState > 1)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_MODE, drdyState < 1, REGMASK_MODE_DRDY_HiZ);
    return true;
  }
}

bool ADS131M04::setPowerMode(uint8_t powerMode)
{
  if (powerMode > 3)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_CLOCK, powerMode, REGMASK_CLOCK_PWR);
    return true;
  }
}

bool ADS131M04::setOsr(uint16_t osr)
{
  if (osr > 7)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_CLOCK, osr << 2 , REGMASK_CLOCK_OSR);
    return true;
  }
}

bool ADS131M04::setChannelEnable(uint8_t channel, uint16_t enable)
{
  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CLOCK, enable << 8, REGMASK_CLOCK_CH0_EN);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CLOCK, enable << 9, REGMASK_CLOCK_CH1_EN);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CLOCK, enable << 10, REGMASK_CLOCK_CH2_EN);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CLOCK, enable << 11, REGMASK_CLOCK_CH3_EN);
    return true;
  }
}

bool ADS131M04::setChannelPGA(uint8_t channel, uint16_t pga)
{
  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_GAIN, pga, REGMASK_GAIN_PGAGAIN0);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_GAIN, pga << 4, REGMASK_GAIN_PGAGAIN1);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_GAIN, pga << 8, REGMASK_GAIN_PGAGAIN2);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_GAIN, pga << 12, REGMASK_GAIN_PGAGAIN3);
    return true;
  }
}

void ADS131M04::setGlobalChop(uint16_t global_chop)
{
  writeRegisterMasked(REG_CFG, global_chop << 8, REGMASK_CFG_GC_EN);
}

void ADS131M04::setGlobalChopDelay(uint16_t delay)
{
  writeRegisterMasked(REG_CFG, delay << 9, REGMASK_CFG_GC_DLY);
}

bool ADS131M04::setInputChannelSelection(uint8_t channel, uint8_t input)
{
  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CH2_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CH3_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
}

bool ADS131M04::setChannelOffsetCalibration(uint8_t channel, int32_t offset)
{

  uint16_t MSB = offset >> 8;
  uint8_t LSB = offset;

  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH0_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH1_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CH2_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH2_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CH3_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH3_OCAL_LSB, LSB << 8 , REGMASK_CHX_OCAL0_LSB);
    return true;
  }
}

bool ADS131M04::setChannelGainCalibration(uint8_t channel, uint32_t gain)
{

  uint16_t MSB = gain >> 8;
  uint8_t LSB = gain;

  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH0_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH1_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CH2_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH2_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CH3_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH3_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
}

bool ADS131M04::isDataReady()
{
  if (digitalRead(ADS131M04_DRDY_PIN) == HIGH)
  {
    return false;
  }
  return true;
}

adcOutput ADS131M04::readADC(void)
{
  uint8_t x = 0;
  uint8_t x2 = 0;
  uint8_t x3 = 0;
  int32_t aux;
  adcOutput res;

  digitalWrite(ADS131M04_CS_PIN, LOW);
  delayMicroseconds(1);

  x = SPI.transfer(0x00);
  x2 = SPI.transfer(0x00);
  SPI.transfer(0x00);

  res.status = ((x << 8) | x2);

  x = SPI.transfer(0x00);
  x2 = SPI.transfer(0x00);
  x3 = SPI.transfer(0x00);

  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    res.ch0 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    res.ch0 = aux;
  }

  x = SPI.transfer(0x00);
  x2 = SPI.transfer(0x00);
  x3 = SPI.transfer(0x00);

  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    res.ch1 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    res.ch1 = aux;
  }

  x = SPI.transfer(0x00);
  x2 = SPI.transfer(0x00);
  x3 = SPI.transfer(0x00);

  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    res.ch2 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    res.ch2 = aux;
  }

  x = SPI.transfer(0x00);
  x2 = SPI.transfer(0x00);
  x3 = SPI.transfer(0x00);

  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    res.ch3 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    res.ch3 = aux;
  }
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(ADS131M04_CS_PIN, HIGH);

  return res;
}

#include <Arduino.h>
#include "BL0940_SPI.hpp"

#define BL0940_DEBUG 1
#if BL0940_DEBUG
#define DBG(...) \
  { Serial.println(__VA_ARGS__); }
#define ERR(...) \
  { Serial.println(__VA_ARGS__); }
#else
#define DBG(...)
#define ERR(...)
#endif /* BL0940_DBG */

BL0940_SPI::BL0940_SPI(uint8_t pin) : _ss_pin(pin)
{
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  SPI.begin();
  
  pinMode(_ss_pin, OUTPUT);
  _pulseSS();
}

BL0940_SPI::~BL0940_SPI()
{
  SPI.end();
}

// TODO: Explicar por qué se usa esta función
void BL0940_SPI::_pulseSS()
{
  digitalWrite(_ss_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_ss_pin, LOW);
}

uint8_t BL0940_SPI::_calcCheckSum(uint8_t *txData, int txLenght, uint8_t *rxData, int rxLenght) 
{
  uint8_t checksum = 0;
  for (int i = 0; i < txLenght; i++) {
    checksum += txData[i];
  }
  for (int i = 0; i < rxLenght; i++) {
    checksum += rxData[i];
  }
  checksum = ~checksum;
  return checksum;
}

void BL0940_SPI::_writeDataSPI(uint8_t *data, size_t len)
{
  uint8_t i=0;
  // uint8_t dataRx[6];
  _pulseSS();
  
  /*
  // TODO: No entiendo lo del 32. Revisar
  while(len-- && i < 6) {
    Serial.println(SPI.transfer(data[i]), HEX);
    i++;  
  }
  // TODO: esto nunca pasaría
  while(i++ < 6) {
    SPI.transfer(0);
  }
  */

  SPI.transfer(data, sizeof(data));
  _pulseSS();
}

bool BL0940_SPI::_writeRegister(uint8_t address, uint32_t data)
{
  //Register Unlock
  uint8_t unlockTxData[6] = { WRITE_OP_ID, USR_WRPROT, 0x55, 0, 0, 0 };
  unlockTxData[5] = _calcCheckSum(unlockTxData, sizeof(unlockTxData)-1, 0, 0);
  _writeDataSPI(unlockTxData, sizeof(unlockTxData));

  //Write Register
  uint8_t txData[6] = { WRITE_OP_ID, address, (uint8_t)(data), (uint8_t)(data >> 8), (uint8_t)(data >> 16) };
  txData[5] = _calcCheckSum(txData, sizeof(txData)-1, 0, 0);
  _writeDataSPI(txData, sizeof(txData));

  return true;
}

bool BL0940_SPI::_readRegister(uint8_t address, uint32_t *data) {
  uint8_t txData[] = { READ_OP_ID, address };
  uint8_t rxData[6] = { READ_OP_ID, address, 0, 0, 0, 0};

  // TODO: Cómo trabaja con punteros hago esto, aunque capaz no es lo mejor
  _writeDataSPI(rxData, sizeof(rxData));

  uint8_t checksum = _calcCheckSum(txData, sizeof(txData), rxData, sizeof(rxData)-1);
  if (rxData[5] != checksum) {
    char message[128];
    Serial.println("Checksum error true");
    sprintf(message, "Checksum error true:%x read:%x.", checksum, rxData[3]);
    ERR(message);
    return false;
  }

  *data = ((uint32_t)rxData[2] << 16) | ((uint32_t)rxData[3] << 8) | (uint32_t)rxData[4];
  return true;
}

bool BL0940_SPI::getCurrentTransientWaveform(float *current)
{
  uint32_t data;
  if (false == _readRegister(I_WAVE, &data)) {
    ERR("Can not read I_WAVE register.");
    return false;
  }

  int32_t rawCurrent = (int32_t)(data << 12) / 4096;
  *current = (float)rawCurrent / ((R5 * 1000.0) / Rt);
  return true;
}

bool BL0940_SPI::getVoltageTransientWaveform(float *voltage)
{
  uint32_t data;
  if (false == _readRegister(V_WAVE, &data)) {
    ERR("Can not read V_WAVE register.");
    return false;
  }

  int32_t rawVoltage = (int32_t)(data << 12) / 4096;
  *voltage = ((float)rawVoltage * (R8 + R9 + R10 + R11 + R12)) / R7;
  return true;
}

bool BL0940_SPI::getCurrentRMS(float *current) {
  uint32_t data;
  if (false == _readRegister(I_RMS, &data)) {
    ERR("Can not read I_RMS register.");
    return false;
  }
  
  *current = ((float)data * Vref) / ((324004.0 * R5 * 1000.0) / Rt);
  return true;
}

bool BL0940_SPI::getVoltageRMS(float *voltage) {
  uint32_t data;
  if (false == _readRegister(V_RMS, &data)) {
    ERR("Can not read V_RMS register.");
    return false;
  }

  *voltage = ((float)data * Vref * (R8 + R9 + R10 + R11 + R12)) / (79931.0 * R7);
  return true;
}

bool BL0940_SPI::getActivePower(float *activePower) {
  uint32_t data;
  if (false == _readRegister(WATT, &data)) {
    ERR("Can not read WATT register.");
    return false;
  }

  int32_t rawActivePower = (int32_t)(data << 8) / 256;
  if (rawActivePower < 0)
    rawActivePower = -rawActivePower;
  *activePower = ((float)rawActivePower * Vref * Vref * (R8 + R9 + R10 + R11 + R12)) / (4046.0 * (R5 * 1000.0 / Rt) * R7);
  return true;
}

bool BL0940_SPI::getActiveEnergy(float *activeEnergy) {
  uint32_t data;
  if (false == _readRegister(CF_CNT, &data)) {
    ERR("Can not read CF_CNT register.");
    return false;
  }

  int32_t rawCF_CNT = (int32_t)(data << 8) / 256;
  if (rawCF_CNT < 0)
    rawCF_CNT = -rawCF_CNT;
  *activeEnergy = ((float)rawCF_CNT * 1638.4 * 256.0 * Vref * Vref * (R8 + R9 + R10 + R11 + R12)) / (3600000.0 * 4046.0 * (R5 * 1000.0 / Rt) * R7);
  return true;
}

bool BL0940_SPI::getPowerFactor(float *powerFactor) {
  uint32_t data;
  if (false == _readRegister(CORNER, &data)) {
    ERR("Can not read CORNER register.");
    return false;
  }

  float rawPowerFactor = cos((2.0 * PI * (float)data * (float)Hz) / 1000000.0) * 100.0;
  if (rawPowerFactor < 0)
    rawPowerFactor = -rawPowerFactor;
  *powerFactor = rawPowerFactor;
  return true;
}

bool BL0940_SPI::getTemperature(float *temperature) {
  uint32_t data;
  if (false == _readRegister(TPS1, &data)) {
    ERR("Can not read TPS1 register.");
    return false;
  }

  int16_t rawTemperature = (int16_t)(data << 6) / 64;
  *temperature = (170.0 / 448.0) * (rawTemperature / 2.0 - 32.0) - 45;
  return true;
}

bool BL0940_SPI::setFrequency(uint32_t Hz) {
  uint32_t data;
  if (false == _readRegister(MODE, &data)) {
    ERR("Can not read MODE register.");
    return false;
  }

  uint16_t mask = 0b0000001000000000;  //9bit
  if (Hz == 50)
    data &= ~mask;
  else
    data |= mask;

  if (false == _writeRegister(MODE, data)) {
    ERR("Can not write MODE register.");
    return false;
  }

  if (false == _readRegister(MODE, &data)) {
    ERR("Can not read MODE register.");
    return false;
  }

  if ((data & mask) == 0) {
    Hz = 50;
    DBG("Set frequency: 50Hz");
  } else {
    Hz = 60;
    DBG("Set frequency: 60Hz");
  }
  return true;
}

bool BL0940_SPI::setUpdateRate(uint32_t rate) {
  uint32_t data;
  if (false == _readRegister(MODE, &data)) {
    ERR("Can not read MODE register.");
    return false;
  }

  uint16_t mask = 0b0000000100000000;  //8bit
  if (rate == 400)
    data &= ~mask;
  else
    data |= mask;

  if (false == _writeRegister(MODE, data)) {
    ERR("Can not write MODE register.");
    return false;
  }

  if (false == _readRegister(MODE, &data)) {
    ERR("Can not read MODE register.");
    return false;
  }

  if ((data & mask) == 0) {
    updateRate = 400;
    DBG("Set update rate: 400ms.");
  } else {
    updateRate = 800;
    DBG("Set update rate: 800ms.");
  }
  return true;
}

bool BL0940_SPI::setOverCurrentDetection(float detectionCurrent) {
  const float magicNumber = 0.72;  // I_FAST_RMS = 0.72 * I_RMS (Values obtained by experiments in the case of resistance load)

  //MODE[12] CF_UNABLE set 1 : alarm, enable by TPS_CTRL[14] configured
  uint32_t data;
  if (false == _readRegister(MODE, &data)) {
    ERR("Can not read MODE register.");
    return false;
  }
  data |= 0b0001000000000000;  //12bit
  if (false == _writeRegister(MODE, data)) {
    ERR("Can not read write register.");
    return false;
  }

  //TPS_CTRL[14] Alarm switch set 1 : Over-current and leakage alarm on
  if (false == _readRegister(TPS_CTRL, &data)) {
    ERR("Can not read TPS_CTRL register.");
    return false;
  }
  data |= 0b0100000000000000;  //14bit  0b0100000000000000
  if (false == _writeRegister(TPS_CTRL, data)) {
    ERR("Can not write TPS_CTRL register.");
    return false;
  }

  //Set detectionCurrent I_FAST_RMS_CTRL
  data = (uint32_t)(detectionCurrent * magicNumber / Vref * ((324004.0 * R5 * 1000.0) / Rt));
  data >>= 9;
  data &= 0x007FFF;
  data |= 0b1000000000000000;  //15bit=1 Fast RMS refresh time is every cycle
  data &= 0x00000000FFFFFFFF;
  if (false == _writeRegister(I_FAST_RMS_CTRL, data)) {
    ERR("Can not write I_FAST_RMS_CTRL register.");
    return false;
  }
  
  float actualDetectionCurrent = (float)(data << 9) * Vref / ((324004.0 * R5 * 1000.0) / Rt);
  char massage[128];
  sprintf(massage, "Set Current Detection: %.1fA.", actualDetectionCurrent);
  DBG(massage);

  return true;
}
  
bool BL0940_SPI::setCFOutputMode() {
  //MODE[12] CF_UNABLE set 0 : alarm, enable by TPS_CTRL[14] configured
  uint32_t data;
  if (false == _readRegister(MODE, &data)) {
    ERR("Can not read MODE register.");
    return false;
  }
  data &= ~0b0001000000000000;  //12bit
  if (false == _writeRegister(MODE, data)) {
    ERR("Can not read write register.");
    return false;
  }
  return true;
}
bool BL0940_SPI::Reset() {

  if (false == _writeRegister(SOFT_RESET, 0x5A5A5A)) {
    ERR("Can not write SOFT_RESET register.");
    return false;
  }
  delay(500);
  return true;
}
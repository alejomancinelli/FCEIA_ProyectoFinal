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

// #define SPI_RX_DEBUG 1

BL0940_SPI::BL0940_SPI(uint8_t pin) : _ss_pin(pin)
{
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

uint8_t BL0940_SPI::_calcCheckSum(uint8_t opCode, uint8_t address, uint8_t *data, size_t dataLen) {
  uint16_t checksum = opCode + address;
  for (size_t i = 0; i < dataLen; i++) {
    checksum += data[i];
  }
  return (uint8_t)(~(checksum & 0xFF));  // Mask and then invert
}

// TODO: Falta probar
void BL0940_SPI::_writeCommand(uint8_t *data, size_t len)
{
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE1)); // up to 900kHz per datasheet
  
  _pulseSS();
  for (size_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);  // Just send, discard response
  }
  _pulseSS();
  
  SPI.endTransaction();
}
void BL0940_SPI::_readCommand(uint8_t *tx, size_t txLen, uint8_t *rx, size_t rxLen)
{
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE1)); // up to 900kHz per datasheet
  
  _pulseSS();
  // Send phase
  for (size_t i = 0; i < txLen; i++) {
    SPI.transfer(tx[i]);
  }

  // Read phase (just send dummy bytes to clock data in)
  for (size_t i = 0; i < rxLen; i++) {
    rx[i] = SPI.transfer(0x00);
  }
  _pulseSS();

  SPI.endTransaction();
}

bool BL0940_SPI::_writeRegister(uint8_t address, uint32_t data)
{
  //Register Unlock
  uint8_t unlockData[3] = { 0x00, 0x00, 0x55 };
  uint8_t unlockChecksum = _calcCheckSum(WRITE_OP_ID, USR_WRPROT, unlockData, 3);
  uint8_t unlockFrame[] = { WRITE_OP_ID, USR_WRPROT, 0x00, 0x00, 0x55, unlockChecksum };
  _writeCommand(unlockFrame, sizeof(unlockFrame));
  
  //Write Register
  uint8_t dataBytes[3] = {
    (uint8_t)(data & 0xFF),        // LSB
    (uint8_t)((data >> 8) & 0xFF), // Mid
    (uint8_t)((data >> 16) & 0xFF) // MSB
  };

  uint8_t checksum = _calcCheckSum(WRITE_OP_ID, address, dataBytes, 3);

  uint8_t writeFrame[] = {
    WRITE_OP_ID,
    address,
    dataBytes[2], // MSB
    dataBytes[1], // MID
    dataBytes[0], // LSB
    checksum
  };

  _writeCommand(writeFrame, sizeof(writeFrame));

  return true;
}

bool BL0940_SPI::_readRegister(uint8_t address, uint32_t *data)
{
  uint8_t txData[] = { READ_OP_ID, address };
  uint8_t rxData[4];  // 3 bytes of data + 1 byte checksum

  _readCommand(txData, sizeof(txData), rxData, sizeof(rxData));

  // Optional: print debug
  #if SPI_RX_DEBUG
  Serial.print("RX: ");
  for (int i = 0; i < sizeof(txData); i++) {
    Serial.printf("%02X ", txData[i]);
  }
  for (int i = 0; i < sizeof(rxData); i++) {
    Serial.printf("%02X ", rxData[i]);
  }
  Serial.println();
  #endif

  // La data parece llegar siempre bien, pero a veces el checksum difiere por 1
  // Por el momento no se tendrá en cuenta
  // uint8_t checksum = _calcCheckSum(READ_OP_ID, address, rxData, 3);
  // if (rxData[3] != checksum) {
  //   Serial.println("Checksum error in _readRegister()");
  //   Serial.printf("Expected: 0x%02X, Received: 0x%02X\n", checksum, rxData[3]);
    
  //   // Probar calcular igualmente para ver si esta mal el checksum
  //   return false;
  // }

  // Convert 3-byte MSB-first data to uint32_t
  *data = ((uint32_t)rxData[0] << 16) | ((uint32_t)rxData[1] << 8) | rxData[2];
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
  
  *current = ((float)data * Vref) / ((324004.0 * R5 * 1000.0) / Rt); // TODO: Puede que el 1000 acá esté mal, revisar bien
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
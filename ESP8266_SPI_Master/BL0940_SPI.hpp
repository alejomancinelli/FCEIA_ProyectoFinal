#ifndef BL0940_SPI_hpp
#define BL0940_SPI_hpp

#include <SPI.h>

#define READ_OP_ID  0x58
#define WRITE_OP_ID 0xA8

/* [--- Electrical parameter register (read only) ---] */
#define I_FAST_RMS      0x00 // Fast current RMS, unsigned
#define I_WAVE          0x01 // Current waveform register, signed 
#define V_WAVE          0x03 // Voltage waveform register, signed
#define I_RMS           0x04 // Current RMS, unsigned 
#define V_RMS           0x06 // Voltage RMS register, unsigned 
#define WATT            0x08 // Active power register, signed 
#define CF_CNT          0x0A // Active energy pulse count, unsigned
#define CORNER          0x0C // Current voltage waveform phase angle register
#define TPS1            0x0E // Internal temperature register, unsigned 
#define TPS2            0x0F // External temperature register, unsigned 

/* [--- User operated register (read and write) ---] */
#define I_FAST_RMS_CTRL 0x10 // Fast current RMS control register
#define I_RMSOS         0x13 // Current RMS offset adjust register 
#define WATTOS          0x15 // Active power offset adjust register 
#define WA_CREEP        0x17 // Active power no-load threshold register
#define MODE            0x18 // User mode selection register 
#define SOFT_RESET      0x19 // When 0x5A5A5A is written, the user area register is reset to default 
#define USR_WRPROT      0x1A // Write protection register.
#define TPS_CTRL        0x1B // Temperature mode control register
#define TPS2_A          0x1C // External temperature sensor gain coefficient adjust register 
#define TPS2_B          0x1D // External temperature sensor offset coefficient adjust register

class BL0940_SPI 
{ 
  private:
    uint8_t _ss_pin;
    void _pulseSS();
    
    uint8_t _calcCheckSum(uint8_t *txData, int txLenght, uint8_t *rxData, int rxLenght);
    
    void _writeDataSPI(uint8_t *data, size_t len);
    
    bool _writeRegister(uint8_t address, uint32_t data);
    bool _readRegister(uint8_t address, uint32_t *data);
    
    // TODO: Checkear
    const uint16_t timeout = 1000;  // Serial timeout[ms]
    const float Vref = 1.218; // [V]
    const float R5 = 3.3;   // [Ohm]
    const float Rt = 2000.0;  // n:1
    const float R8 = 20.0;  // [kOhm]
    const float R9 = 20.0;  // [kOhm]
    const float R10 = 20.0;  // [kOhm]
    const float R11 = 20.0;  // [kOhm]
    const float R12 = 20.0;  // [kOhm]
    const float R7 = 24.9;  // [Ohm]
    uint16_t Hz = 50;   // [Hz]
    uint16_t updateRate = 400; // [ms]

  public:
    BL0940_SPI(uint8_t pin);
    ~BL0940_SPI();

    // TODO: Test
    bool getCurrentTransientWaveform( float *current ); // [A]
    bool getVoltageTransientWaveform( float *voltage ); // [V]

    bool getCurrentRMS( float *current );  // [A]
    bool getVoltageRMS( float *voltage );  // [V]
    bool getActivePower( float *activePower );  // [W]
    bool getActiveEnergy( float *activeEnergy );  // [kWh]
    bool getPowerFactor( float *powerFactor );  // [%]
    bool getTemperature( float *temperature );  // [deg C]
    bool setFrequency( uint32_t Hz = 60 );  // 50 or 60  [Hz]
    bool setUpdateRate( uint32_t rate = 400 );  // 400 or 800  [ms]
    bool setOverCurrentDetection( float detectionCurrent = 15.0 );  // [A] CF pin is high if current is larger than detectionCurrent
    bool setCFOutputMode(); // Enegy pulse output CF pin
    bool Reset();
};

#endif /* BL0940_SPI */
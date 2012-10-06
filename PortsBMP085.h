// 2009-02-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

/// @file
/// Port library interface to BMP085 sensors connected via I2C.
/// See http://jeelabs.net/projects/hardware/wiki/pp1

/// Interface for the Pressure Plug - see http://jeelabs.org/pp
class BMP085 : public DeviceI2C {
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
    
    uint16_t readWord(uint8_t last) const
        { uint16_t v = read(0) << 8; return v | read(last); }
    void readFromReg(uint8_t reg) const
        { send(); write(reg); receive(); }
            
public:
    enum { TEMP, PRES };
    int32_t meas[2];
    uint8_t oss;

    /// Constructor for BMP085 class.
    /// @param p I2C port to be used.
    /// @param osrs 0..3 Oversampling setting.
    BMP085 (const PortI2C& p, uint8_t osrs =0)
        : DeviceI2C (p, 0x77), oss (osrs) {}

    /// Set the oversampling setting for the high resolution mode.
    /// @param osrs 0..3.
    void setOverSampling(uint8_t osrs) { oss = osrs; }

    /// Start a measurement.
    /// @param type BMP::TEMP for temperature or BMP::PRES for pressure.
    uint8_t startMeas(uint8_t type) const;
    /// Get the results from the last measurement.
    /// @param type BMP::TEMP for temperature or BMP::PRES for pressure.
    int32_t getResult(uint8_t type);

    /// Take a measurement.
    /// @param type BMP::TEMP for temperature or BMP::PRES for pressure.
    int32_t measure(uint8_t type)
        { delay(startMeas(type)); return getResult(type); }

    void getCalibData();
    void calculate(int16_t& tval, int32_t& pval) const;
};

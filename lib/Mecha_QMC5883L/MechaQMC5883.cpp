#include "MechaQMC5883.h"

#include <Wire.h>

void MechaQMC5883::setAddress(uint8_t addr) {
    address = addr;
}

void MechaQMC5883::WriteReg(byte Reg, byte val) {
    Wire.beginTransmission(address); //start talking
    Wire.write(Reg); // Tell the HMC5883 to Continuously Measure
    Wire.write(val); // Set the Register
    Wire.endTransmission();
}

void MechaQMC5883::init() {
    WriteReg(0x0B, 0x01);
    //Define Set/Reset period
    setMode(Mode_Continuous, ODR_200Hz, RNG_8G, OSR_512);
    /*
    Define
    OSR = 512
    Full Scale Range = 8G(Gauss)
    ODR = 200HZ
    set continuous measurement mode
     */
}

void MechaQMC5883::setMode(uint16_t mode, uint16_t odr, uint16_t rng, uint16_t osr) {
    WriteReg(0x09, mode | odr | rng | osr);
}

void MechaQMC5883::softReset() {
    WriteReg(0x0A, 0x80);
}

/**
 * read values from device
 * @return status value:
 *  - 0:success
 *  - 1:data too long to fit in transmit buffer
 *  - 2:received NACK on transmit of address
 *  - 3:received NACK on transmit of data
 *  - 4:other error
 *  - 8:overflow (magnetic field too strong)
 */
int MechaQMC5883::readRaw(int* x, int* y, int* z) {
    Wire.beginTransmission(address);
    Wire.write(0x00);
    int err = Wire.endTransmission();
    if (err) {
        return err;
    }
    Wire.requestFrom(address, 7);
    *x = (int) (int16_t) (Wire.read() | Wire.read() << 8);
    *y = (int) (int16_t) (Wire.read() | Wire.read() << 8);
    *z = (int) (int16_t) (Wire.read() | Wire.read() << 8);
    byte overflow = Wire.read() & 0x02;
    return overflow << 2;
}

int MechaQMC5883::readRaw(int* x, int* y, int* z, int* a) {
    int err = readRaw(x, y, z);
    *a = azimuth(y, x);
    return err;
}

int MechaQMC5883::readCorrected(int* x, int* y, int*z) {
    int rawX, rawY, rawZ;
    int err = readRaw(&rawX, &rawY, &rawZ);
    if (err) {
        return err;
    }
    
    // Subtract offset to remove hard iron errors
    rawX -= (x_min + x_max) / 2.0;
    rawY -= (y_min + y_max) / 2.0;
    rawZ -= (z_min + z_max) / 2.0;

    // Scale the ellipse on each axis to correct for soft iron errors
    rawX *= x_scale;
    rawY *= y_scale;
    rawZ *= z_scale;

    *x = rawX;
    *y = rawY;
    *z = rawZ;
    
    return err;
}


void MechaQMC5883::setMinMaxCalibration(int minX, int minY, int minZ, int maxX, int maxY, int maxZ) {
    x_min = minX;
    y_min = minY;
    z_min = minZ;
    x_max = maxX;
    y_max = maxY;
    z_max = maxZ;
    calcCalibration();
}

void MechaQMC5883::calcCalibration() {
    // http://www.camelsoftware.com/2016/03/13/imu-maths-calculate-orientation-pt3/

    float vmaxX = x_max - ((x_min + x_max) / 2.0);
    float vmaxY = y_max - ((y_min + y_max) / 2.0);
    float vmaxZ = z_max - ((z_min + z_max) / 2.0);


    float vminX = x_min - ((x_min + x_max) / 2.0);
    float vminY = y_min - ((y_min + y_max) / 2.0);
    float vminZ = z_min - ((z_min + z_max) / 2.0);

    float avgsX = vmaxX + (vminX*-1);
    float avgsY = vmaxY + (vminY*-1);
    float avgsZ = vmaxZ + (vminZ*-1);

    avgsX = avgsX / 2.0;
    avgsY = avgsY / 2.0;
    avgsZ = avgsZ / 2.0;

    float avg_rad = avgsX + avgsY + avgsZ;
    avg_rad /= 3.0;

    x_scale = (avg_rad / avgsX);
    y_scale = (avg_rad / avgsY);
    z_scale = (avg_rad / avgsZ);

}

float MechaQMC5883::azimuth(int *a, int *b) {
    float azimuth = atan2((int) *a, (int) *b) * 180.0 / PI;
    return azimuth < 0 ? 360 + azimuth : azimuth;
}

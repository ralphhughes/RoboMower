#ifndef Mecha_QMC5883
#define Mecha_QMC5883

#include "Arduino.h"
#include "Wire.h"

#define QMC5883_ADDR 0x0D


//REG CONTROL

//0x09

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000


class MechaQMC5883{
public:


void setAddress(uint8_t addr);

void init(); //init qmc5883

void setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr); // setting

void softReset(); //soft RESET

int readRaw(int* x,int* y,int* z); //reading
int readRaw(int* x,int* y,int* z,int* a);
int readCorrected(int* x,int* y,int* z); //reading

float azimuth(int* a,int* b);

void setMinMaxCalibration(int minX, int minY, int minZ, int maxX, int maxY, int maxZ);

private:

void WriteReg(uint8_t Reg,uint8_t val);
void calcCalibration();


uint8_t address = QMC5883_ADDR;
int x_min, y_min, z_min;
int x_max, y_max, z_max;
float x_scale, y_scale, z_scale;

};



#endif

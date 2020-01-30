TI INA219 hi-side i2c current/power monitor Library for Arduino
===============================================================
# Authors
## John De Cristofaro
* Initial releases

## Jukka-Pekka Sarjanen
* Quite much small changes to Johns's original lib mostly documentation and
C++ "mambo jambo" Doxy documentation etc. 
* One major change / fix:
calibrate() function fixed to give correct ratings.

## gandy92
* Fix compiling issue due to case sensitivity on linux systems

## Flavius Bindea
* More examples
* added shuntCurrentRaw()
* changed begin() and default values for ADAFRUIT INA219 breakout
* Added rewrite modes, read conversion ready, overflow functions
* Fixed bug in calibrate() in order to use class variable cal

# Tests and fonctionalities
* Tested at standard i2c 100kbps signaling rate.
* This library does not handle triggered conversion modes. It uses the INA219
in continuous conversion mode. All reads are from continous conversions.
* A note about the gain (PGA) setting:
	The gain of the ADC pre-amplifier is programmable in the INA219, and can
	be set between 1/8x (default) and unity. This allows a shunt voltage 
	range of +/-320mV to +/-40mV respectively. Something to keep in mind,
	however, is that this change in gain DOES NOT affect the resolution
	of the ADC, which is fixed at 1uV. What it does do is increase noise
	immunity by exploiting the integrative nature of the delta-sigma ADC.
	For the best possible reading, you should set the gain to the range
	of voltages that you expect to see in your particular circuit. See
	page 15 in the datasheet for more info about the PGA.

# Known bugs:
* may return unreliable values if not connected to a bus or at bus currents below 10uA.

# Dependencies:
* Arduino Wire library

# Usage
## Basic
Look at [ina219_test](examples/ina219_test/ina219_test.ino) exemple sketch.
This example works out of the box for Adafruit's INA219 Breakout.

Include defintions and define needed object:
```Arduino
#include <Wire.h>
#include <INA219.h>

INA219 monitor;
```

In the `setup()` function initialise the INA219:
```Arduino
monitor.begin();
```

Then in the `loop()` function make calls to different functions that are returning the values:
```Arduino
  Serial.print("raw shunt voltage: ");
  Serial.println(monitor.shuntVoltageRaw());
  
  Serial.print("raw bus voltage:   ");
  Serial.println(monitor.busVoltageRaw());
  
  Serial.println("--");
  
  Serial.print("shunt voltage: ");
  Serial.print(monitor.shuntVoltage() * 1000, 4);
  Serial.println(" mV");
  
  Serial.print("shunt current: ");
  Serial.print(monitor.shuntCurrent() * 1000, 4);
  Serial.println(" mA");
  
  Serial.print("bus voltage:   ");
  Serial.print(monitor.busVoltage(), 4);
  Serial.println(" V");
  
  Serial.print("bus power:     ");
  Serial.print(monitor.busPower() * 1000, 4);
  Serial.println(" mW");
```
## Enhanced setup
If you want to use a different setup or if you do not use the Adafruit's breakout then in the `setup()` function you need to call `configure()` and `calibrate()`.

An exemple is in [ina219_test_nondefault](examples/ina219_test_nondefault/ina219_test_nondefault.ino).

Extract of the `setup()` function:
```Arduino
  monitor.begin();

  // setting up our configuration
  monitor.configure(INA219::RANGE_16V, INA219::GAIN_2_80MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);
  
  // calibrate with our values
  monitor.calibrate(SHUNT_R, SHUNT_MAX_V, BUS_MAX_V, MAX_CURRENT);
```

## functions
All the functions are well comented in [INA219.h](INA219.h) and [INA219.cpp](INA219.cpp)
* `begin()`
starts the communication with the INA219 and does the default setup.
* `configure()`
setups the INA219 mode.

  The args are: `(range, gain, bus_adc, shunt_adc, mode)`
  * range : Range for bus voltage
    * RANGE_16V : Range 0-16 volts
    * RANGE_32V (default):  Range 0-32 volts
  * gain : Set Gain for shunt voltage (choose the lowest possible depending on your hardware)
    * GAIN_1_40MV : 40mV
    * GAIN_2_80MV : 80mV
    * GAIN_4_160MV : 160mV
    * GAIN_8_320MV (default): 320mV
  * bus_adc : Configure bus voltage conversion
    * ADC_9BIT : 9bit, converion time  84us.
    * ADC_10BIT : 10bit, converion time 148us.
    * ADC_11BIT : 11bit, converion time 2766us.
    * ADC_12BIT (default): 12bit converion time 532us.
    * ADC_2SAMP : 2 samples converion time 1.06ms.
    * ADC_4SAMP : 4 samples converion time 2.13ms.
    * ADC_8SAMP : 8 samples converion time 4.26ms.
    * ADC_16SAMP : 16 samples converion time 8.51ms
    * ADC_32SAMP : 32 samples converion time 17.02ms.
    * ADC_64SAMP : 64 samples converion time 34.05ms.
    * ADC_128SAMP : 128 samples converion time 68.10ms.
  * shunt_adc: Configure shun voltage conversion. Same values as for bus_adc
  * mode: Sets operation mode.
    * PWR_DOWN : Power Down
    * ADC_OFF
    * CONT_SH : Shunt Continuous
    * CONT_BUS : Bus Continuous
    * CONT_SH_BUS (default): Shunt and Bus, Continuous.  
* `calibrate(r_shunt, v_shunt_max, v_bus_max, i_max_expected)` function is doing the calculations as described in the INA219 datasheet and sets up the calibration registers.

  The args are:
  * r_shunt : Value of shunt in Ohms.
  * v_shunt_max : Maximum value of voltage across shunt.
  * v_bus_max : Maximum voltage of bus.
  * i_max_expected : Maximum current draw of bus + shunt.
* `reconfig()` rewrites the last used config to the INA219.
* `recalibrate()` rewrites the calibration registers as they were calculated by `calibrate()` function.
* `ready()` returns the value of the ready bit. It's value is updated according to the last call to `busVoltageRaw()` or `busVoltage()`.

      based on the datasheet page 30:
          Although the data from the last conversion can be read at any time,
          the INA219 Conversion Ready bit (CNVR) indicates when data from
          a conversion is available in the data output registers.
          The CNVR bit is set after all conversions, averaging, 
          and multiplications are complete.
          CNVR will clear under the following conditions:
          1.) Writing a new mode into the Operating Mode bits in the 
              Configuration Register (except for Power-Down or Disable)
          2.) Reading the Power Register
          
      page 15:
          The Conversion Ready bit clears under these
          conditions:
          1. Writing to the Configuration Register, except
          when configuring the MODE bits for Power Down
          or ADC off (Disable) modes;
          2. Reading the Status Register;
          3. Triggering a single-shot conversion with the
          Convert pin.
* `overflow()` returns the value of the overflow bit. It's value is updated according to the last call to `busVoltageRaw()` or `busVoltage()`.
* `reset()` : Resets the INA219.
* `shuntVoltageRaw()` : Returns the raw binary value of the shunt voltage
* `busVoltageRaw()` :  Returns raw bus voltage binary value.
* `shuntCurrentRaw()` :  Returns raw bus voltage binary value.
* `shuntVoltage()` : Returns the shunt voltage in volts.
* `busVoltage()` : Returns the bus voltage in volts.
* `shuntCurrent()` : Returns the shunt current in amps.
* `busPower()` : Returns the bus power in watts

# Licence
MIT license

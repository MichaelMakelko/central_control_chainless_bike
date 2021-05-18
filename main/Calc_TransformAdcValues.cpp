#include "Calc_TransformAdcValues.h"


////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Value / Time Variables
static int voltage = 0;           // 10 Bit value [0 ...1023] transformed -> voltage [mV]
static long signed int current = 0;           // 10 Bit value [0 ...1023] transformed -> current [mA]
static double percentage = 0.0;   // 10 Bit value [0 ...1023] transformed -> percentage (0 ... 1) [-]
static int power = 0;             // 10 Bit value [0 ...1023] transformed -> power [W]




////// GET (READ) / SET (WRITE) FUNCTIONS //////            

//// Voltage ////

// Get Function: 10bit Variable transformed to Voltage [1500 mV .. 4500 mV]
int get_10BitTransformedTo3000mV(int adc10BitDigital) {    // TODO update the variable
  voltage = adc10BitDigital * 2.93 + 1500;                      // transformationFactor = 3000 mV / 1024 bit = 2.93 mV/bit
  return voltage;                                               // + 1500 mV to offset from [0 mV .. 3000 mV] -> [1500 mV .. 4500 mV]
}


// Get Function: 10bit Variable transformed to Voltage [0 mV .. 5000mV]
int get_10BitTransformedTo5000mV(int adc10BitDigital) {
  voltage = adc10BitDigital * 4.88;                             // transformationFactor = 5000 mV / 1024 bit = 4.88 mV/bit
  return voltage;                   
}


// Get Function: 10bit Variable transformed to Voltage [0 mV .. 50000mV]
int get_10BitTransformedTo50000mV(int adc10BitDigital) {
  voltage = adc10BitDigital * 48.83;                            // transformationFactor = 50000 mV / 1024 bit = 48.83 mV/bit
  return voltage;                   
}



//// Percentage ////

// Get Function: 10bit Variable transformed to Percentage [0 .. 1]
double get_10BitTransformedTo1(int adc10BitDigital) {
  percentage = adc10BitDigital * 0.098 * 0.01;                  // transformationFactor = 100 / 1024 bit = 0.098 1/bit
  return percentage;                                            // * 0.01 to transfrom unit from [0% .. 100%] -> [0 .. 1]
}



//// Power ////

// Get Function: 10bit Variable transformed to Power [50 W .. 320 W]
int get_10BitTransformedTo270W(int adc10BitDigital) {
  power = adc10BitDigital * 0.264 + 50;                         // transformationFactor = 270 W / 1024 bit = 0.264 W/bit
  return power;                                                 // + 50 W to offset from [0 W .. 270 W] -> [50 W .. 320 W]
}



//// Current ////

// Transform the 10 Bit Variable of the current Current to a voltage equivalent to the Current in the next timestep
signed int get_10BitTransformedTo60000mA(int adc10BitDigital) {

  //// 10bit Variable transformed to Voltage [0 mV .. 5000 mV] ////
  voltage = get_10BitTransformedTo5000mV(adc10BitDigital);


  //// Voltage [0 mV .. 5000 mV] transformed to Current [-30000 mV .. 5000 mV] ////
  // If voltage <= 520 mV -> current = 0 mA
  if (voltage <= 520) {
    current = 0;
  }

  // else if voltage >= 4480 mV -> currrent = 60000mA
  else if (voltage >= 4480) {
    current = 60000;
  }

  // else -> calculate the specific current with 0 mA at 520 mV and 60000 mA at 4480 mV
  else {
    current = voltage * 15.152;   // transformationFactor = 60000 mA / 3960 mV = 15.152 mA/mV
  }                                                             

  current -= 30000;               // - 30000 mA to offset from [0 mA .. 60000 mA] -> [-30000 mA .. 30000 mA]
  return current;
}

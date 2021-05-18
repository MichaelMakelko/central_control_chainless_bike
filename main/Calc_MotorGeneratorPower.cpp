#include "Calc_MotorGeneratorPower.h"


////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Value / Time Variables
static long int powerActual = 0;  // Actual power which is provided to the motor currently [W]
static int powerTarget = 0;       // Target power out of Poti Motor and Poti Throttle which should be controlled [W]



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

// Get Function: Generator Power out of Voltage and Current = Actual Power
long int get_powerOutOfVoltageAndCurrent(unsigned int voltage, long int current) {
  powerActual = powerActual = 0.000001 * voltage * current;   // P = U * I   [mV * mA = W * 10^-6]
  return powerActual;                                         //    * 0.000001 to transform the unit from [W * 10^-6] -> [W]
}


// Get Function: Power out of maximal Power restriction and throttle position = Target Power
int get_powerOutOfMaxValueAndThrottlePosition(int maxPower, double throttlePosition, double adjustment) {
  powerTarget = maxPower * throttlePosition * adjustment;         // [W] * [-] * [-] = [W]
  return powerTarget;
}

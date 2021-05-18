#include "PWM_MotorPowerControlling.h"


////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Constants / Threshold Variables
static double velocityLimit_forwardLinearAdjustment = 6.94;   // 25 km/h transformed -> [km/h] / 3.6 = [m/s]
static double velocityLimit_forwardNoPower = 7.5;             // 27 km/h transformed -> [km/h] / 3.6 = [m/s]
static double velocityLimit_reverseNoPower = 1.6;             // 6 km/h transformed -> [km/h] / 3.6 = [m/s]
static double threshold_linearAdjustment = 0.5556;            // 2 km/h transformed -> [km/h] / 3.6 = [m/s]

static int threshold_controlValueMaxLimit = 1599;     // ICR4 (configured in PWM_PedallingWithstand) = TOP value Timer 4 -> [0 mV ... 5000 mV] [mV]
static int threshold_500mV = 160;                     // 1/10 factor of ICR4 -> [0 mV ... 500 mV]

static double Kp = 0.8;   // Proportional Factor of PID Controller [-] - due to no available Hardware set up to 1 [-]
static double Ki = 0.0;   // Intagral Factor of PID Controller [-] - due to no available Hardware set up to 1 [-]

// Value / Time Variables
static double velocityDifference = 0.0;       // Difference between the actual vehicleVelocity and velocityLimit_linearAdjustment [m/s]
static double factor_linearAdjustment = 0.0;  // factor to adjust the Power Output of the motor, depening by status_drive and vehicleVelocity -> (0 ... 1) [-]

static double delta = 0.0;      // difference between actual and target value of PID Controller of the current timestep [unit of input variable]
static double sumDelta = 0.0;   // sum of the difference between actual and target value of PID Controller over all timesteps [unit of input variable]

static double amount_p = 0.0;   // Proportional amount of new control value [unit of input variable]
static double amount_i = 0.0;   // Integrational amount of new control value [unit of input variable]

static int controlValue = 0;    // Output of the PI Controller - control value depending by the actual value and the target value [unit of input variable]

static double OCR4B_pwmValue = 0;   // OCR4B / on_pwmDutyCycle to adjust the voltage at the PWM Pin7 / PH4 - PWM voltage between [1500 mV ... 4500 mV] [-]
                                


////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

// Get Function: Factor Linear Adjustment to decrease the Power of the Motor up from a certain speed at a specific status_drive
double get_linearMotorPowerAdjustment(int status_drive, double vehicleVelocity) {

  // If the vehicle is in status_drive = forward drive && velocity > 27 km/h -> set motor Power to 0
  if (status_drive == 3 && vehicleVelocity >= velocityLimit_forwardNoPower){
    factor_linearAdjustment = 0;
  }

  // If the vehicle is in status_drive = forward drive && velocity > 25 km/h -> calculate factor_linearAdjustment to reduce the Motor Power
  else if (vehicleVelocity >= velocityLimit_forwardLinearAdjustment){
    velocityDifference =  vehicleVelocity - velocityLimit_forwardLinearAdjustment;
    factor_linearAdjustment = 1 - threshold_linearAdjustment * velocityDifference;
  }

  // If the vehicle is in status_drive = reverse drive && velocity > 6 km/h -> set motor Power to 0
  else if (status_drive == 6 && vehicleVelocity >= velocityLimit_reverseNoPower) {
    factor_linearAdjustment = 0;
  }

  // Validation case to check values: COMMENT DURING NORMAL RUN
  else {
    factor_linearAdjustment = 1;
  }

  return factor_linearAdjustment;
}


// Get Function: value of PI Controler out of actual and target value
int get_PiControllerValue(double actualValue, double targetValue) {
  delta = targetValue - actualValue;    // determine difference between actual and target value
  sumDelta += delta;                    // sum of delta over all timesteps - restriction can be added as Anti-Windup (avoids overshooting)

  amount_p = delta * Kp;      // proportional amount of new control value
  amount_i = sumDelta * Ki;   // integration amount of new control value

  controlValue = actualValue + amount_p + amount_i;   // new contol value
  return controlValue;
}


// Get Function: Current OCR4B value to adjust the Motor Power (voltage at the specific PWM pin) - PWM voltage between [1500 mV ... 4500 mV]
double get_motorPwmControlValue(double power_motor) {                                                                              
  OCR4B_pwmValue = (threshold_500mV * 3 + (threshold_500mV * 6 * (power_motor/320))) - 1; // on_pwmDutyCycle = threshold_1500mV + (range_pwmDutyCycle * (current_power / max_power)) -> [-] * [W]/[W] = [-]
                                                                                          // range_pwmDutyCycle = ICR4 / 10 * 6 -> describes PWM voltage [1500 mV ... 4500 mV]
                                                                                          //      ICR4 (configured in PWM_PedallingWithstand) = TOP value Timer 4 = complete_pwmDutyCycle = 1599 -> describes PWM Voltage [0 mV ... 5000 mV]
  return OCR4B_pwmValue;
}

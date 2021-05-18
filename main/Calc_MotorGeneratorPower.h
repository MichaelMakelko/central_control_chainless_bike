#ifndef CALC_MOTOR_GENERATOR_POWER_H     // if not CALC_MOTOR_GENERATOR_POWER_H (identifier was defined) -> true
#define CALC_MOTOR_GENERATOR_POWER_H 


////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

long int get_powerOutOfVoltageAndCurrent(unsigned int voltage, long int current);

int get_powerOutOfMaxValueAndThrottlePosition(int maxPower, double throttlePosition, double adjustment);


#endif

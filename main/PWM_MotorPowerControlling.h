#ifndef PWM_MOTOR_POWER_CONTROLLING_H     // if not PWM_MOTOR_POWER_CONTROLLING_H (identifier was defined) -> true
#define PWM_MOTOR_POWER_CONTROLLING_H


////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

double get_linearMotorPowerAdjustment(int status_drive, double vehicleVelocity);

int get_PiControllerValue(double actualValue, double targetValue);

double get_motorPwmControlValue(double power_motor);


#endif

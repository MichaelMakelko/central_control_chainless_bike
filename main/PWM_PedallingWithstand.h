#ifndef PWM_PEDALLING_WITHSTAND_H     // if not PWM_PEDALLING_WITHSTAND_H  (identifier was defined) -> true
#define PWM_PEDALLING_WITHSTAND_H 


////// INITIALISATION FUNCTIONS //////

void Init_PWM_PedalingWithstand_MotorPower(void);



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

double get_generatorPwmControlValue(int voltage_potiGenerator,int voltage_inputGenerator,int voltage_outputMotor);


#endif

#ifndef ADC_MOTOR_GENERATOR_THROTTLE_H     // if not ADC_MOTOR_GENERATOR_THROTTLE_H (identifier was defined) -> true
#define ADC_MOTOR_GENERATOR_THROTTLE_H


////// INITIALISATION FUNCTIONS //////

void Init_ADC_MotorGeneratorThrottle();



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

void set_startAdConversion(void);
     
int get_flagAdConversionCompleted(void);

int *get_adConversionResults(void);   


#endif

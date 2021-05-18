#include "PWM_PedallingWithstand.h"
#include <avr/io.h>
#include <avr/interrupt.h>


////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Value / Time Variables
static int OCR4A_pwmValue = 0;    // OCR4A / on_pwmDutyCycle to adjust the voltage at the PWM Pin6 / PH3 - PWM voltage between [0 mV ... 5000 mV] [-]



////// INITIALISATION FUNCTIONS //////
void Init_PWM_PedalingWithstand_MotorPower(void){

  // Declare Output-Pins
  DDRH |= (1 << PH3);     // Generator: OR condition: set bit 3 of Port H to 1 = Output (Pedaling Withstand)
  DDRH |= (1 << PH4);     // Motor: OR condition: set bit 4 of Port H to 1 = Output (Power Motor)
  
  
  // Configurate PWM with Timer 4
  ICR4   = 1599;        // TOP value counter of Timer 4 (lowering PWM-frecuency caused by lower ICR value of Timer 4)
                        // f_pwm_counter = 16Mhz/(N *(ICR4+1)) = 10 kHz   (+1, because 0 needs to be count)
                        //        N = 1 ... Prescaler (siehe CS30 - CS32)
  TCCR4A = (1<<COM4A1)|(1<<COM4B1)|(1<<WGM41);  // PWM Output (COM), appearance PWM-signal (WGM)
                                                // COM4A0=0; COM4A1=1 -> Generator: PWM non-inverting Mode at Arduino Pin 6 (as seen in "Compare Output Mode, Fast PWM" Table 17-4)
                                                //                                  (shift to 5V till OC4B afterwards 0V till end of period / duty cycle)
                                                // COM4B0=0; COM4B1=1 -> Motor: PWM non-inverting Mode at Arduino Pin 7 (as seen in "Compare Output Mode, Fast PWM" Table 17-4)
                                                //                              (shift to 5V till OC4B afterwards 0V till end of period / duty cycle)
  TCCR4B = (1<<WGM43) |(1<<WGM42) | (1<<CS40);  // appearance PWM-signal (WGM), frequency PWM-signal (CS)
                                                // WGM10=0; WGM11=1; WGM12=1; WGM13=1 -> Mode 14 with Timer 4
                                                //                                    -> Fast PWM with ICR4 as TOP-value
                                                // CS40=1; CS41=0; CS42=0 -> Prescaler of Timer 4 = 1 (no prescaling)
                                                //                        -> Frequency of counting mechanism of Timer 4 = 16 MHz
  OCR4A = 0;    // Generator: TOP / Limit value of PWM counting mechanism - until which one 5V will be applied (Arduino Pin 6 / PH3)
  OCR4B = 0;    // Motor: TOP / Limit value of PWM counting mechanism - until which one 5V will be applied (Arduino Pin 7 / PH4)
}



////// GET (READ) / SET (WRITE) FUNCTIONS //////      

// Get Function: Current OCR4A value to adjust the peadaling withstand (voltage at the specific PWM pin) - PWM voltage between [0 mV ... 5000 mV]
double get_generatorPwmControlValue(int voltage_potiGenerator,int voltage_inputGenerator, int voltage_outputMotor) {
    OCR4A_pwmValue = voltage_inputGenerator / (1599 * (voltage_outputMotor + voltage_potiGenerator)); // on_pwmDutyCycle = U_inputAccumulator / (U_ouputAccumulator + U_potiGenerator) * complete_pwmDutyCycle -> [mV] / ([mV] + [mV]) * [-] = [-]
                                                                                                      // complete_pwmDutyCycle = TOP Value Timer 4 = ICR4 = 1599 -> describes PWM voltage [0 mV ... 5000 mV]
    return OCR4A_pwmValue; 
}

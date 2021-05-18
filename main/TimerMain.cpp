#include "TimerMain.h"
#include <avr/io.h>
#include <avr/interrupt.h>


////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR (Interrupt-Function)

// Interrupt Variables
static volatile int flag_timestepCompleted = 0;   // 0 = Timer3 is still running; 1 = Timer3 reached time limit, next cycle should start [-]



////// INITIALISATION FUNCTIONS //////
void Init_TimerMain(void){
  TCCR3A= 0;                          // COM3A0 = 0; COM3A1 = 0 -> normal Port operation - external Interrupt OC3A disconnected
                                      // COM3B0 = 0; COM3B1 = 0 -> normal Port operation - external Interrupt OC3B disconnected
  TCCR3B= (1 << WGM32)|(1 << CS31);   // WGM30 = 0; WGM31 = 0; WGM32 = 1 -> waveform generation mode 4 - reserved (nothing specified in the manual)
                                      // CS30 = 1; CS31 = 1; CS32 = 0 -> internal Processor clock cycle (16 MHz) / 8 
                                      //                              -> 16MHz/8 = 2 MHz -> 0,0005 ms
  TIMSK3= (1 << OCIE3A);              // TOIE3 = 0 -> Interrupt is not acitvated and executed with overflow (8 bit Timer = 256; 16 bit Timer = 65536)
                                      // OCIE3A = 1 -> Interrupt is acitvated and executed with arrival of OCR3A - comparison value
                                      // OCIE3B = 0 -> Interrupt is not acitvated and executed with arrival of OCR3B - comparison value
  OCR3A = 19999;                      // TOP value of Timer3 (could count until 65536 - 16 bit Timer)
                                      // -> Cycle Time Interrupt = 2000 timesteps * (1/2MHz) = 10 ms
  TCNT3  = 0;                         // BOTTOM value of Timer 3 - in general 0  

  set_flagNewTimestep();
}



////// INTERRUPT FUNCTIONS //////
ISR(TIMER3_COMPA_vect) {      // Interrupt becomes triggered, when Timer3 matches OCR3A in Normal-Mode (each 10 ms)
  flag_timestepCompleted = 1;
}

  

////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

// Get Function Timer3:  is Timer3 running? - 0 = running; 1 = reached its time limit
int get_flagTimestepCompleted(void) {
  return flag_timestepCompleted;
}


// Set Function Timer3: Cycle Time next Iteration
void set_flagNewTimestep(void) {
  flag_timestepCompleted = 0;
}

#include "Valid_LedRuntime.h"
#include <avr/io.h>


////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Status / Flag Variables
static int flag_thresholdRuntimeExceeded = 0;   // Flag which indicates whether the threshold of runtime was exceeded - LED lights up when its exceeded [-]



////// INITIALISATION FUNCTIONS //////
void Init_Valid_LedRuntime(void){     // External Interrupt - exectues at input Signal / input Voltage on specified Pin
  
  // Declare Output-Pin
  DDRH |= (1 << PH1);       // OR condition: set bit 1 of Port H (PH5) to 1 = Output

  // Pulled down to 0V (PORT = write)
  PORTH &= ~(1 << PH1);    // AND condition: set bit 1 of Port H to 0 = 0V (Pin 8 = external LED)
}



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

// Get Function: 
int get_thresholdRuntimeExceeded(long int threshold_runtime, int time_cycleTime) {
  if (time_cycleTime > threshold_runtime) {
    flag_thresholdRuntimeExceeded = 1;
    PORTH |= (1 << PH1);    // OR condition: set bit 1 of Port H to 1 = 5V (Pin 16 = external LED) -> LED = ON
  }
  else {
    flag_thresholdRuntimeExceeded = 0;
    PORTH &= ~(1 << PH1);   // AND condition: set bit 1 of Port H to 0 = 0V (Pin 16 = external LED) -> LED = OFF
  }

  return flag_thresholdRuntimeExceeded;
}

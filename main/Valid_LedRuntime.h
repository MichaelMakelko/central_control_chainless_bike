#ifndef VALID_LED_RUNTIME_H     // if not VALID_LED_RUNTIME_H  (identifier was defined) -> true
#define VALID_LED_RUNTIME_H 


////// INITIALISATION FUNCTIONS //////

void Init_Valid_LedRuntime(void);



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

int get_thresholdRuntimeExceeded(long int threshold_runtime, int time_cycleTime);     // Validation Function


#endif

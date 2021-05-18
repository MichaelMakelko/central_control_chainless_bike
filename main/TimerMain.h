#ifndef TIMER_MAIN_H     // if not TIMER_MAIN_H (identifier was defined) -> true
#define TIMER_MAIN_H


////// INITIALISATION FUNCTIONS //////

void Init_TimerMain(void);



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

int get_flagTimestepCompleted(void);

void set_flagNewTimestep(void);


#endif

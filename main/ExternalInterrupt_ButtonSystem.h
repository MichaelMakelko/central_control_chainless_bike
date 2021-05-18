#ifndef EXTERNAL_INTERRUPT_BUTTON_SYSTEM_H     // if not EXTERNAL_INTERRUPT_BUTTON_SYSTEM_H (identifier was defined) -> true
#define EXTERNAL_INTERRUPT_BUTTON_SYSTEM_H


////// INITIALISATION FUNCTIONS //////

void Init_ExternalInterrupt_ButtonSystem(void);



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

int get_statusSystem(int flag_firstInterrupt, int status_system, int status_drive);

int get_flagErrorStatusSystem(void);

void set_flagErrorStatusSystem(void);



////// VALIDATION FUNCTIONS //////

int get_timestepButtonPressed(void);  

int get_flagButtonSystemPressed(void);  

int get_statusButtonSystemPressed(void);


#endif

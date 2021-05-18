#ifndef EXTERNAL_INTERRUPT_REED_VELOCITY_H     // if not EXTERNAL_INTERRUPT_REED_VELOCITY_H (identifier was defined) -> true
#define EXTERNAL_INTERRUPT_REED_VELOCITY_H


////// INITIALISATION FUNCTIONS //////

void Init_ExternalInterrupt_ReedVelocity(void);



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

double get_vehicleVelocity(long int time_now);

int get_driveStatus(int status_system, int status_pedaling, double vehicleVelocity);



////// VALIDATION FUNCTIONS //////

int get_totalNumberSignals(void);

int get_timestepSameDriveStatus(void);


#endif

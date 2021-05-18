#include "ExternalInterrupt_ReedVelocity.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>


////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Interrupt Variables
static volatile int flag_reedSignal = 0;          // Flag to show whether reed signal was received [-]

// Status / Flag Variables
static int status_drive = 0;                      // Status drive mode: 0 = Parking; 1 = sale, 2 = ForwardDrive; 3 = Recuperation; 4 = ReverseDrive [-]
static int flag_status_drive = 0;                 // Flag of all status_drive to indicate the status_drive of the current timestep: 0 = Parking; 1 = sale, 2 = ForwardDrive; 3 = Recuperation; 4 = ReverseDrive [-]
static int flag_status_drive_old = 0;             // Flag of all status_drive to indicate the status_drive of the last timestep: 0 = Parking; 1 = sale, 2 = ForwardDrive; 3 = Recuperation; 4 = ReverseDrive [-]

// Counter Variables
static int i_timestepsLastInterrupt = 0;          // timestep counter since last interrupt [unit TimerMain]
static int i_totalNumberSignals = 0;              // Counter of all recieved signals from the external Interrupt [-]
static int i_timestepSameDriveStatus = 0;         // timestep counter of current status_drive [unit TimerMain]

// Constants / Threshold Variables
static int threshold_timestepsTillSwitch = 100;   // delay until status_drive switch is performed -> threshold_timestepsTillSwitch * cycleTime main [unit TimerMain]
static int threshold_timestepsTillStopped = 200;  // delay factor until vehicle_veloctiy will be set to 0 without occurance of an Interrupt -> threshold_timestepsTillStopped * TimerMain cycleTime [unit TimerMain]
static double wheel_circumference = 1.5959;       // U = 20 inch wheel * Pi [m]

// Value / Time Variables
static long int time_last = 0.0;                  // old time / last time - reed contact sent signal [nsec]
static long int time_duration = 0.0;              // difference between current time and last time [msec]
static double vehicleVelocity = 0.0;              // average vehicle velocity during last wheel rotation [m/s]    // TODO transform to int with a.e. * 100 to int (usage of int to save ressources)



////// INITIALISATION FUNCTIONS //////
void Init_ExternalInterrupt_ReedVelocity(void){
  
  // Declare Input-Pins
  DDRD &= ~(1 << INT3);     // AND condition: set bit 3 of Port D to 0 = Input - processes Interrupt at Input-signal
                            // (Pin 18 / PD3 = external interrupt at signal from button)
  
  // Declare Output-Pins
  DDRH |= (1 << PH5);       //PH5 as Output set ,as digital signal to show the status of Recuperation , 5V -> deactivated ,0V-> activated
  DDRH |= (1 << PH6);       //PH6 as Output set ,as digital signal to show the status of ReverseDrive , 5V -> deactivated ,0V-> activated
  
  // Activate Pull-Up - pulled up to 5V (PORT = write)
  PORTD |= (1 << INT3);     // OR condition: set bit 3 of Port D to 1 = 5V (Pin 18 = external interrupt button)
  
  // Configurate external Interrupt                        
  EICRA |= (1<<ISC30) | (1<<ISC31);   // Interrupt at rising edge
  EIMSK |= (1<<INT3);                 // activation external Interrupt INT3
}



////// INTERRUPT FUNCTIONS //////
ISR(INT3_vect) {            // External Interrupt - exectues at input Signal / input Voltage on specified Pin      
  flag_reedSignal = 1;      // becomes triggered when the reed contact completes another wheel rotation
}



////// GET (READ) / SET (WRITE) FUNCTIONS //////                          

// Get Function: current vehicle Velocity
double get_vehicleVelocity(long int time_now) {
  i_timestepsLastInterrupt++;
  
  // If timesteps since the last Interrupt happened > threshold for the Vehicle has probably stopped -> set vehicleVelocity = 0
  if (i_timestepsLastInterrupt > threshold_timestepsTillStopped) {
    vehicleVelocity = 0;
  }

  // If the Interrupt was executed -> calculate new vehicleVelocity
  if (flag_reedSignal == 1) {
    time_duration = 65 * (time_now - time_last) >> 16;                // Time in Millisecond
    vehicleVelocity = (1000 * wheel_circumference) / time_duration;   // Calculation new current vehicle veloicity

    flag_reedSignal = 0;
    time_last = time_now;
    i_timestepsLastInterrupt = 0;
  }

  // Switch the vehicleVelocity depending in which direction the vehicle is driving
  switch (status_drive) {
    case 0:     // parking mode: status_drive = 0
      vehicleVelocity = fabs(vehicleVelocity);
      break;
    case 1:     // sale forward mode: status_drive = 1
      vehicleVelocity = fabs(vehicleVelocity);
      break;
    case 2:     // sale backward mode: status_drive = 2
      vehicleVelocity = fabs(vehicleVelocity) * -1;
      break;
    case 3:     // driving forward mode: status_drive = 3
      vehicleVelocity = fabs(vehicleVelocity);
      break;
    case 4:     // recuperation forward mode: status_drive = 4
      vehicleVelocity = fabs(vehicleVelocity);
      break;
    case 5:     // recuperation backward mode: status_drive = 5
      vehicleVelocity = fabs(vehicleVelocity) * -1;
      break;
    case 6:     // driving in reverse mode: status_drive = 6
      vehicleVelocity = fabs(vehicleVelocity) * -1;
      break;
  }

  return vehicleVelocity;
}


// Get Function: current drive status
int get_driveStatus(int status_system, int status_pedaling, double vehicleVelocity) {

  // if status_drive didnt change -> increase timestep counter
  if (flag_status_drive == flag_status_drive_old) {
    i_timestepSameDriveStatus++;
  }
  else {
    i_timestepSameDriveStatus = 0;
  }
  flag_status_drive_old = flag_status_drive;
  
  // if vehicle ist standing && no pedalling rotaiton -> parking mode: status_drive = 0
  if (vehicleVelocity == 0 && status_pedaling == 0) {
    flag_status_drive = 0;
    if (i_timestepSameDriveStatus > threshold_timestepsTillSwitch) {
      status_drive = 0;
      }
  }

  // if vehicle velocity > 0 && no pedaling roation -> sale forward mode: status_drive = 1
  else if (status_system == 1 && vehicleVelocity > 0 && status_pedaling == 0) {
    flag_status_drive = 1;
    if (i_timestepSameDriveStatus > threshold_timestepsTillSwitch) {
      status_drive = 1;
      }
  }

    // if vehicle velocity < 0 && no pedaling roation -> sale backward mode: status_drive = 2
  else if (status_system == 1 && vehicleVelocity < 0 && status_pedaling == 0) {
    flag_status_drive = 2;
    if (i_timestepSameDriveStatus > threshold_timestepsTillSwitch) {
      status_drive = 2;
      }
  }

  // if vehicle velocity >= 0 && pedalling forward -> driving forward mode: status_drive = 3
  else if (status_system == 1 && vehicleVelocity >= 0 && status_pedaling == 1) {
    flag_status_drive = 3;
    if (i_timestepSameDriveStatus > threshold_timestepsTillSwitch) {
      status_drive = 3;
      }
  }

  // if vehicle velocity > 0 && pedalling backward -> recuperation forward mode: status_drive = 4
  else if (status_system == 1 && vehicleVelocity > 0 && status_pedaling == 2) {
    flag_status_drive = 4;
    if (i_timestepSameDriveStatus > threshold_timestepsTillSwitch) {
      status_drive = 4;
      PORTH &= ~(1<<PH5);   // PH5 Output on, digital signal to activate Recuperation , low   
      }
  }

  // if vehicle velocity < 0 && pedalling forward -> recuperation backward mode: status_drive = 5
  else if (status_system == 1 && vehicleVelocity < 0 && status_pedaling == 1) {
    flag_status_drive = 5;
    if (i_timestepSameDriveStatus > threshold_timestepsTillSwitch) {
      status_drive = 5;
      PORTH &= ~(1<<PH5);   // PH5 Output on,digital signal to activate Recuperation , low   
      }
  }

  // if vehicle velocity < 0 && pedalling backward -> driving in reverse mode: status_drive = 6
  else if (status_system == 1 && vehicleVelocity <= 0 && status_pedaling == 2) {
    flag_status_drive = 6;
    if (i_timestepSameDriveStatus > threshold_timestepsTillSwitch) {
      status_drive = 6;
      PORTH &= ~(1<<PH6);   // PH6 Output on,digital signal to activate ReverseDrive , low
      }
  }

  // If recuperation mode (status_drive = 4 or 5) is activated -> set specific output pin to 5V - deactivate reverse mode
  if(status_drive != 4 || status_drive != 5) {
     PORTH |= (1<<PH5);    // deactivate the Recuperation ,low -> high
  }

  // If reverse mode (status_drive = 6) is activated -> set specific output pin to 5V - deactivate recuperation mode
  else if(status_drive != 6 ) {
     PORTH |= (1<<PH6);     // deactivate the ReverseDrive  ,low -> high
  }
  
  return status_drive;
}



////// VALIDATION FUNCTIONS //////

// Get Function: current number button_pushed   // Function to check the functionality of the code
int get_totalNumberSignals(void) {
  if (flag_reedSignal == 1) {
    i_totalNumberSignals++;
  }
  return i_totalNumberSignals;
}


// Get_Function: Timestep since beeing in a new status_drive to first switch after a certain time (avoiding switching each ms caused by noise)
int get_timestepSameDriveStatus(void) {
  return i_timestepSameDriveStatus;
}

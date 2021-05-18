#include "ExternalInterrupt_ButtonSystem.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>


////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Interrupt Variables
static int flag_buttonPressed = 0;                    // Flag to indicate whether the Interrupt was executed [-]

// Status / Flag Variables
static int flag_firstInterrupt = 0;                   // Flag which avoids to perform the procedure after first Interrupt while Launching the System [-]
static int status_buttonPressed = 0;                  // Contains the status of the Pin - 5V = HIGH / 0V = LOW [-]
static int flag_errorStatusSystem = 0;                // Flag to indicate whether the System Button was pressed while driving - status_drive != 0 [-]

// Counter Variables
static int i_timestepButtonPressed = 0;               // Counter for the timesteps since the System Button is pressed [unit TimerMain]

// Constants / Threshold Variables
static int threshold_timestepsSystemShutdown = 100;   // Threshold for the number of timesteps to shutdown the system [unit TimerMain]



////// INITIALISATION FUNCTIONS //////
void Init_ExternalInterrupt_ButtonSystem(void){     // External Interrupt - exectues at input Signal / input Voltage on specified Pin
  
  // Declare Input-Pins
  DDRD &= ~(1 << PD2);    // AND condition: set bit 2 of Port D (PD2) to 0 = Input
  DDRD &= ~(1 << INT2);   // AND condition: acitvate  -  processes Interrupt at Input-signal

  // Activate Pull-Ups - pulled up to 5V (PORT = write)
  PORTD |= (1 << PD2);    // OR condition: set bit 2 of Port D (PD2) to 1 = 5V
  PORTD |= (1 << INT2);   // OR condition: set bit 2 of Port D (PD2) to 1 = 5V (Pin 19 = external interrupt button)

  // Configurate external Interrupts                 
  EICRA |= (0<<ISC20) | (1<<ISC21);   // Interrupt at falling edge
  EIMSK |= (1<<INT2);                 // activation external Interrupt INT2       
}



////// INTERRUPT FUNCTIONS //////

// System Button Status Change
ISR(INT2_vect) {            // becomes triggered when the System Button to start or shutdown the System becomes pressed 
  flag_buttonPressed = 1;
}



////// GET (READ) / SET (WRITE) FUNCTIONS //////               

// Get Function: current status_system
int get_statusSystem(int flag_firstInterrupt, int status_system, int status_drive) {

  // If System was just started (time_now = 0) -> dont start the system after executing the Interrupt
  if (flag_firstInterrupt == 0) {
    return status_system = 0;
  }
  
  // if Interrupt was triggered
  if (flag_buttonPressed == 1) {
      status_buttonPressed = (PIND & (1 << PD2)) / 4;

    // if System Button is pushed && System wasnt launched yet && vehicle is in Parking mode -> launch System
    if ((status_buttonPressed == 0) && (status_system == 0) && (status_drive == 0)) {
      status_system = 1;   
      flag_buttonPressed = 0;
    }

    // if System Button is pressed && System is launched already && vehicle is in Parking mode -> increase timestep counter of pressing System Button
    else if ((status_buttonPressed == 0) && (status_system == 1) && (status_drive == 0)) {
      i_timestepButtonPressed++;
    }

    // if System Button isnt pressed anymore -> set back timestep counter to 0
    else if (status_buttonPressed == 1) {
      flag_errorStatusSystem = 0;
      i_timestepButtonPressed = 0;
      flag_buttonPressed = 0;
    }

    // else Error System Message
    else {
      flag_errorStatusSystem = 1;
    }

    // if System Button is pressed for longer then Threshold -> shutdown System
    if (i_timestepButtonPressed > threshold_timestepsSystemShutdown) {
      status_system = 0;
      i_timestepButtonPressed = 0;
      flag_buttonPressed = 0;
    }

    return status_system;
  }
}


// Get Function: get an Error flag when the System Button was pressed while driving
int get_flagErrorStatusSystem() {
  return flag_errorStatusSystem;
}


// Set Function: set back Error flag to 0 
void set_flagErrorStatusSystem(void) {
  flag_errorStatusSystem = 0;
}



////// VALIDATION FUNCTIONS //////

// Get Function: current count of timestep since the System Button was pressed
int get_timestepButtonPressed(void) {
  return i_timestepButtonPressed;
}


// Get Function: current flag of the System Button
int get_flagButtonSystemPressed(void) {
  return flag_buttonPressed;
}


// Get Function: current status of the System Button
int get_statusButtonSystemPressed(void) {
  return status_buttonPressed;
}

#include "ExternalInterrupts_EncoderPedaling.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>


////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Interrupt Variables
static volatile int flag_encoderChannelA = 0;     // Flag what indicates that status of Encoder Channel A has changed: 0 = no status changed, 1 = status change [-]
static volatile int flag_encoderChannelB = 0;     // Flag what indicates that status of Encoder Channel B has changed: 0 = no status changed, 1 = status change [-]

// Status / Flag Variables
static int encoderChannelA = 0;                   // Status of Encoder Channel A: 0 = LOW/0V;, 1 = High/5V [-]
static int encoderChannelB = 0;                   // Status of Encoder Channel B: 0 = LOW/0V;, 1 = High/5V [-]
static int status_pedaling = 0;                   // Status peddaling mode: 0 = no pedaling, 1 = forward padeling, 2 = backward padeling [-]

// Counter Variables
static int i_timestepsLastInterrupt = 0;          // timestep counter since last interrupt [unit TimerMain]

// Constants / Threshold Variables
static int threshold_timestepsTillNoPadeling = 100;    // Timesteps until status_pedaling will be set to 0 without occurance of an Interrupt [unit TimerMain]

// Value / Time Variables
static long int time_last = 0;                    // old time / last time - reed contact sent signal [nsec]
static long int time_duration = 0;                // difference between current time and last time [nsec]
static signed int rotational_speed = 0;           // average rotational speed during last pedal rotation [1/min]



////// INITIALISATION FUNCTIONS //////
void Init_ExternalInterrupts_EncoderPedaling(void){
  
  // Declare Input-Pins
  DDRE &= ~(1 << PE4);    // AND condition: set bit 4 of Port E (PE4) to 0 = Input
  DDRE &= ~(1 << INT4);   // AND condition: set bit 4 of Port E to 0 = Input - processes Interrupt at Input-signal (Pin 2 / PE4 = external interrupt at signal from button)
  DDRE &= ~(1 << PE5);    // AND condition: set bit 5 of Port E (P54) to 0 = Input
  DDRE &= ~(1 << INT5);   // AND condition: set bit 5 of Port E to 0 = Input - processes Interrupt at Input-signal (Pin 3 / PE5 = external interrupt at signal from button)

  // Activate Pull-Ups - pulled up to 5V (PORT = write)
  PORTE |= (1 << PE4);    // OR condition: set bit 4 of Port E to 1 = 5V
  PORTE |= (1 << INT4);   // OR condition: set bit 4 of Port E to 1 = 5V (Pin 2 = external interrupt button)
  PORTE |= (1 << PE5);    // OR condition: set bit 5 of Port E to 1 = 5V
  PORTE |= (1 << INT5);   // OR condition: set bit 5 of Port E to 1 = 5V (Pin 3 = external interrupt button)

  // Configurate external Interrupts                 
  EICRB |= (1<<ISC40) | (0<<ISC41);   // Interrupt at any edge
  EIMSK |= (1<<INT4);                 // activation external Interrupt INT4       
  EICRB |= (1<<ISC50) | (0<<ISC51);   // Interrupt at any edge
  EIMSK |= (1<<INT5);                 // activation external Interrupt INT5
}



////// INTERRUPT FUNCTIONS //////

// Encoder Channel A
ISR(INT4_vect) {              // External Interrupt - exectues at input Signal / input Voltage on specified Pin 
  flag_encoderChannelA = 1;   // becomes triggered when Encoder Channel A gets a Signal
}

// Encoder Channel B
ISR(INT5_vect) {              // External Interrupt - exectues at input Signal / input Voltage on specified Pin 
  flag_encoderChannelB = 1;   // becomes triggered when Encoder Channel B gets a Signal
}



////// GET (READ) / SET (WRITE) FUNCTIONS //////               

// Get Function: current Pedaling Rotational Speed
int get_pedalingRotationalSpeed(long int time_now) {
  
  // If the Interrupt of encoderChannelA or encoderChannelB was beeing executed -> calculate new rotational_speed
  if (flag_encoderChannelA == 1 || flag_encoderChannelB == 1) {
    time_duration = 65 * (time_now - time_last) >> 16  ;          // time in millisecond
    rotational_speed = 60000 / (time_duration * 4) ;              // RPM ,how many cycles in 1 minute
    time_last = time_now;
  }

  // Switch the rotational_speed depending how the pedals are beeing rotated -> no pedaling = 0; forwared pedalin = +n; backward pedaling = -n
  switch (status_pedaling) {
    case 0:     // no pedaling
      rotational_speed = 0;
      break;
    case 1:     // forward pedaling
      rotational_speed = abs(rotational_speed);
      break;
    case 2:     // backward pedaling
      rotational_speed = abs(rotational_speed) * -1;
      break;
  }
  return rotational_speed;
}


// Get Function: Pedaling Direction
int get_statusPedaling(void) {

  // no padeling - time delay since last Interrupt
  i_timestepsLastInterrupt++;
  if (i_timestepsLastInterrupt > threshold_timestepsTillNoPadeling) {     //after x us * 3 /100*100ms = 3000ms = 3 s it wil be triggered
    status_pedaling = 0;
  }
  
  // Encoder Channel A
  if (flag_encoderChannelA == 1) {
    encoderChannelA = (PINE & (1 << PE4)) >> 4;
    
    // forward padeling - rising edge 
    if (encoderChannelA == 1 && encoderChannelB == 0) {
      status_pedaling = 1;
    }
    // forward padeling - falling edge 
    else if (encoderChannelA == 0 && encoderChannelB == 1) {
      status_pedaling = 1;
    }
    
    // backward padeling - rising edge 
    else if (encoderChannelA == 1 && encoderChannelB == 1) {
      status_pedaling = 2;
    }
    // backward padeling - falling edge 
    else if (encoderChannelA == 0 && encoderChannelB == 0) {
      status_pedaling = 2;
    }
    
    // Error Handling - in Case encoderChannelA or encoderChannelB gets a value != 0 or 1
    else {
      status_pedaling = 0;
    }
    flag_encoderChannelA = 0;
    i_timestepsLastInterrupt = 0;
  }

  // Encoder Channel B
  if (flag_encoderChannelB == 1) {
    encoderChannelB = (PINE & (1 << PE5)) >> 5;

    // forward padeling - rising edge 
    if (encoderChannelB == 1 && encoderChannelA == 1) {
      status_pedaling = 1;
    }
    // forward padeling - falling edge 
    else if (encoderChannelB == 0 && encoderChannelA == 0) {
      status_pedaling = 1;
    }

    // backward padeling - rising edge 
    else if (encoderChannelB == 1 && encoderChannelA == 0) {
      status_pedaling = 2;
    }
    // forward padeling - falling edge 
    else if (encoderChannelB == 0 && encoderChannelA == 1) {
      status_pedaling = 2;
    }

    // Error Handling - in Case encoderChannelA or encoderChannelB gets a value != 0 or 1
    else {
      status_pedaling = 0;
    }
    flag_encoderChannelB = 0;
    i_timestepsLastInterrupt = 0;
  }

  return status_pedaling;
}



////// VALIDATION FUNCTIONS //////

// Get Function: Pedaling Encoder Channel A value
int get_pedalingEncoderChannelA(void) {
  return(encoderChannelA);
}


// Get Function: Pedaling Encoder Channel B value
int get_pedalingEncoderChannelB(void) {
  return(encoderChannelB);
}

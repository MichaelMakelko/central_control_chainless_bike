#include "ADC_MotorGeneratorThrottle.h"
#include <avr/io.h>
#include <avr/interrupt.h>



////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Interrupt Variables
#define numberChannels 7
static volatile int status_conversionChannelFinished = 0;                     // status at which Channel the conversion was finished last - number matches the actual converted AD Channel [-]
static volatile int flag_conversionAllChannelsFinished = 0;                   // flag whether all Channels were converted [-]
static volatile int i_channel = 0;                                            // counter of AD channels (which have been converted already) [-]
static volatile int channel_conversionResults[numberChannels];                // values of all ADCs [0 ... 1023] - this timestep [-]
static volatile int channel_number[numberChannels] = {0, 1, 2, 3, 4, 5, 6};   // Channels / Pins of the ADCs [-]



////// INITIALISATION FUNCTIONS //////
void Init_ADC_MotorGeneratorThrottle() {
 
  // Configuraton ADC and its Interrupt
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1 | (1 << ADPS0));  // ADEN=1 ... activate ADC
                                                                                    // ADIE=1 ... activate Interrupt
                                                                                    // ADPS0=1, ADPS1=1, ADPS2=1 ... Pre-Scaler of ADC conversion = 128
                                                                                    //                               -> 16MHz/128 = 125KHz conversion frequency
  ADMUX  = (1 << ADLAR) | (1 << REFS0);   // ADLAR=1 ... Conversion results will be saved from left to right in right in Register (ADLAR=0 Standard)
                                          // REFS0=1, REFS1=0 ... Internal Reference Voltage (AREF) = Supply Voltage 5V (AVCC)
                                          // MUX0=0, MUX1=0, MUX2=0, MUX3=0, MUX4=0, MUX5=0 ... selected Input Channel = ADC0 (no differential input or gain)

  // Start 1st AD Conversion to initialize ADC
  ADCSRA |= (1 << ADSC);    // ADSC=1 ... start AD conversion - when conversion is complete -> ADSC=0
}



////// INTERRUPT FUNCTIONS //////
ISR(ADC_vect) {         //Interrupt of the ADC - becomes triggered, when ADSC is set to 1
  
  // Read the conversion results of all channels
  channel_conversionResults[status_conversionChannelFinished] = ADCW;

  // If the number of the next Channel to convert is bigger than the max number of Channels -> set back to start converting from Channel 0 again
  status_conversionChannelFinished++;
  if (status_conversionChannelFinished >= numberChannels) {
    status_conversionChannelFinished = 0;

    ADMUX = (1 << REFS0) | (channel_number[status_conversionChannelFinished] & ((1 << MUX2) | (1 << MUX1) | (1 << MUX0)) );   // set back ADMUX to convert Channel 0 next (due to set back status_conversionChannelFinished)
                                                                                                                              // -> REFS0=1, REFS1=0 ... Internal Reference Voltage (AREF) = Supply Voltage 5V (AVCC)
                                                                                                                              // -> MUX next Channel Number = 0 & 0b00000111 = 0b00000000
    flag_conversionAllChannelsFinished = 1;
  } 

  // Else -> convert the next Channel
  else {
    ADMUX = (1 << REFS0) | (channel_number[status_conversionChannelFinished] & ((1 << MUX2) | (1 << MUX1) | (1 << MUX0)) );   // set forward ADMUX to convert the next channel next (due to incrementing status_conversionChannelFinished)
                                                                                                                              // -> REFS0=1, REFS1=0 ... Internal Reference Voltage (AREF) = Supply Voltage 5V (AVCC)
                                                                                                                              // -> MUX next Channel Number = next Channel number & 0b00000111
                                                                                                                              //    a.e. next Channel number = 3: 00000011 & 0b00000111 = 0b00000011
    ADCSRA |= (1 << ADSC);                                                                                                    // ADSC=1 ... start AD conversion - when conversion is complete -> ADSC=0 
    flag_conversionAllChannelsFinished = 0;
  }
}



////// GET (READ) / SET (WRITE) FUNCTIONS //////

// Set Function: Start conversion of all Channels
void set_startAdConversion(void) {
  flag_conversionAllChannelsFinished = 0;   // set flag to conversion not finished for this timestep
  ADCSRA |= (1 << ADSC);                    // ADSC=1 ... start AD conversion - when conversion is complete -> ADSC=0 
}


// Get Function: flag_adConversionCompleted whether conversion is finished already of the current timestep
int get_flagAdConversionCompleted(void) {
  return flag_conversionAllChannelsFinished;
}


// Get Function: current converted channel values - [ADC0, ADC1, ADC2, ADC3, ADC4, ADC5, ADC6]
int *get_adConversionResults(void) {        // Usage of a pointer for the memcpy function
  return (channel_conversionResults);
}

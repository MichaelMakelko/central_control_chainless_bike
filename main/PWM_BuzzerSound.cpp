// calculation the Noten-value: (16 MHz/64*Frequenz)-1 Note = Wert ICR1
// BSP C-Dur: (16MHz/64*264Hz)-1 = 946
// Quelle Notenfrequenzen: https://kilchb.de/muslekt1.php

#include "PWM_BuzzerSound.h"
#include <avr/io.h>
#include <avr/interrupt.h>


////// INITIALISATION VARIABLES //////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Interrupt Variables
static volatile long int timeToPlay = 0;      // time to play in 1/10 sec
static volatile long int i_timePlayed = 0;    // Counter indicates the number of the PWM-Interrupts which is reached so far
static volatile int flag_toneIsPlaying = 0;   // Flag Indicates whether a tone is playing currently

// Status / Flag Variables
static int status_system = 0;   // Status System: 0 = shutdown, 1 = launched   
static int status_drive = 0;    // Status drive mode: 0 = Parking; 1 = sale forward; 2 = sale backward; 3 = Forward drive; 4 = Recuperation forward; 5 = Recuperation backward; 6 = Reverse drive

// Counter Variables
static int i_tone = 0;    // counter of the currently played tone of the currently playing melody

// Constants / Threshold Variables
static int numberTones = 8;            // max number of tones of each melody to play

long int frequency_toneC = 953;        // C-Dur has a frequence of 262 Hz [1/s]
long int frequency_toneD = 841;        // D-Dur has a frequence of 297 Hz [1/s]
long int frequency_toneE = 757;        // E-Dur has a frequence of 330 Hz [1/s]
long int frequency_toneF = 709;        // F-Dur has a frequence of 352 Hz [1/s]
long int frequency_toneG = 1275;       // G-Dur has a frequence of 196 Hz [1/s]
long int frequency_toneA = 1135;       // A-Dur has a frequence of 220 Hz [1/s]
long int frequency_toneB = 1011;       // B-Dur has a frequence of 247 Hz [1/s]
long int frequency_toneO = 1249;       // O-Dur has a frequence of 200 Hz [1/s]

static char tones_statusSystem0[9] = {0, 'C', 'G', 'G', 'A', 'G', 0, 'B', 'C'};   // Melody, when status_system == activated [-]
static char tones_statusSystem1[9] = {0, 'C', 'B', 'A', 'G', 0, 0, 0, 0};         // Melody, when status_system == deactivated [-]
static char tones_systemError[9] = {0, 'B', 'B', 'B', 'B', 'B', 0, 0, 0};         // Melody, when status_system == deactivated [-]
static char tones_statusDrive0[9] = {0, 'C', 'D', 'E', 'C', 0, 0, 0, 0};          // Melody, when status_drive == Parking [-]
static char tones_statusDrive1[9] = {0, 'C', 'C', 'C', 'D', 'E', 0, 0, 0};        // Melody, when status_drive == Saling forward [-]
static char tones_statusDrive2[9] = {0, 'C', 'C', 'C', 'D', 'E', 0, 0, 0};        // Melody, when status_drive == Saling backward [-]
static char tones_statusDrive3[9] = {0, 'A', 0, 'E', 0, 'A', 0, 'E', 0};          // Melody, when status_drive == Forward drive [-]
static char tones_statusDrive4[9] = {0, 'E', 'D', 'C', 'B', 'A', 0, 0, 0};        // Melody, when status_drive == Recuperation forward [-]
static char tones_statusDrive5[9] = {0, 'E', 'D', 'C', 'B', 'A', 0, 0, 0};        // Melody, when status_drive == Recuperation backward [-]
static char tones_statusDrive6[9] = {0, 'A', 'C', 'E', 0, 0, 0, 0, 0};            // Melody, when status_drive == Reverse drive [-]

static int toneDurations_statusSystem0[9] = {0, 20, 10, 10, 20, 20, 20, 20, 20};  // tone durations of each Tone in status_system == activated [unit TimerMain]
static int toneDurations_statusSystem1[9] = {0, 20, 10, 10, 20, 0, 0, 0, 0};      // tone durations of each Tone in status_system == deactivated [unit TimerMain]
static int toneDurations_systemError[9] = {0, 10, 10, 10, 10, 10, 0, 0, 0};       // tone durations of each Tone during status_systemError == 1 [unit TimerMain]
static int toneDurations_statusDrive0[9] = {0, 20, 20, 20, 20, 0, 0, 0, 0};       // tone durations of each Tone in status_drive == Parking [unit TimerMain]
static int toneDurations_statusDrive1[9] = {0, 20, 20, 20, 20, 20, 0, 0, 0};      // tone durations of each Tone in status_drive == Saling forward [unit TimerMain]
static int toneDurations_statusDrive2[9] = {0, 20, 20, 20, 20, 20, 0, 0, 0};      // tone durations of each Tone in status_drive == Saling backward [unit TimerMain]
static int toneDurations_statusDrive3[9] = {0, 20, 20, 20, 0, 0, 0, 0, 0};        // tone durations of each Tone in status_drive == Forward drive [unit TimerMain]
static int toneDurations_statusDrive4[9] = {0, 10, 10, 10, 10, 10, 10, 10, 0};    // tone durations of each Tone in status_drive == Recuperation forward [unit TimerMain]
static int toneDurations_statusDrive5[9] = {0, 10, 10, 10, 10, 10, 10, 10, 0};    // tone durations of each Tone in status_drive == Recuperation backward [unit TimerMain]
static int toneDurations_statusDrive6[9] = {0, 10, 10, 10, 10, 10, 0, 0, 0};      // tone durations of each Tone in status_drive == Reverse drive [unit TimerMain]


 
////// INITIALISATION FUNCTIONS //////
void Init_PWM_BuzzerSound(void) {     // TODO Buzzer needs to have its own Timer, since the TimerMain is to short for hearing a melody
  
  // Declare Output-Pin
  DDRB |= (1<<PB5);       // OR condition: set bit 5 of Port B (PB5) to 1 = Output

  // Configurate PWM with Timer 1
  TCCR1A = (1<<COM1A1)|(1<<WGM11);    // PWM Output (COM), appearance PWM-signal (WGM)
                                      // COM1A0 = 0; COM1A1 = 1 -> PWM non-inverting Mode at Arduino Pin 11 (as seen OC1A in circuit diagram)
                                      //                        -> (shift to 5V till OCR1A afterwards 0V till end of period / cycle)
  TCCR1B = (1<<WGM13)|(1<<WGM12) | (1<<CS10) | (1<<CS11);   // appearance PWM-signal (WGM, WGM), frequency PWM-signal (CS)
                                                            // WGM10 = 0; WGM11 = 1; WGM12 = 1; WGM13 = 1 -> Mode 14 with Timer1
                                                            //                                            -> Fast PWM with ICR1 as TOP-value
                                                            // CS10 = 1; CS11 = 1; CS12 = 0 -> Prescaler of Timer1 = 64
                                                            //                              -> Frequency of counting mechanism of Timer1 = 16 MHz / 64 = 0.25 MHz
  TIMSK1= (1 << TOIE1);   // TOIE1 = 1 -> Timer/Counter Overflow interrupt is enabled
  ICR1 = 1;               // TOP value counter - lowering PWM-frecuency caused by lower ICR value of Timer1
                          // f_pwm_counter = 16Mhz/(N*(ICR1+1)) = 125 kHz   (+1, because 0 needs to be count)
                          //        N = 64 ... Prescaler (as seen in CS30 - CS32)
  OCR1A = 0;              // Limit value OCR of the PWM counting mechanism - until which one 5V will be applied (Arduino Pin 11)
}



////// INTERRUPT FUNCTIONS //////            
ISR(TIMER1_OVF_vect) {      // Interrupt of Timer 1 - will be ecexuted when Timer 1 reaches TOP value ICR1                                                       
  
  // If a Tone is playing -> counter for played time becomes incremented
  if (flag_toneIsPlaying) {
    i_timePlayed++;

    // If the counter for played time reaches the defined time to play -> the current Tone will stop playing
    if (i_timePlayed >= timeToPlay) {
      OCR1A = 0;                // set ouput voltage of PWM Pin to 0
      ICR1 = 1;                 // set TOP value of counter to adjust frequency of PWM Signal to 1
      flag_toneIsPlaying = 0;   
      i_timePlayed = 0;
      timeToPlay = 0;
    }
  }
}



////// LOCAL FILE FUNCTIONS //////

// play tone in current position of melody
void playTone(char melodyTone, int toneLength) {  // Timer with a cycle Time of 1/10 sec is required 
  switch(melodyTone){
    // C-Dur
    case 'C':   
      ICR1 = frequency_toneC;                     // Adjust TOP value of counter to adjust frequency of PWM Signal to determine the right tone
      OCR1A = ICR1 >> 1;                          // Adjust ouput voltage of PWM Pin to set up volume of PWM signal -> volume is always 50 Procent
      timeToPlay = 666 * toneLength >> 8;         // 666 * time to play the tone >> 8   ->   2,6 * time to play the tone = duration of how long tone is going to play
      break;
    // D-Dur
    case 'D':   
      ICR1 = frequency_toneD;
      OCR1A = ICR1 >> 1;
      timeToPlay = 768 * toneLength >> 8;
      break;
    // E-Dur
    case 'E':   
      ICR1 = frequency_toneE;
      OCR1A = ICR1 >> 1;
      timeToPlay = 845 * toneLength >> 8;
      break;
    // F-Dur
    case 'F':   
      ICR1 = frequency_toneF;
      OCR1A = ICR1 >> 1;
      timeToPlay = 896 * toneLength >> 8;
      break;
    // G-Dur
    case 'G':   
      ICR1 = frequency_toneG;
      OCR1A = ICR1 >> 1;
      timeToPlay = 1024 * toneLength >> 8;
      break;
    // A-Dur
    case 'A':   
      ICR1 = frequency_toneA;
      OCR1A = ICR1 >> 1;
      timeToPlay = 1126 * toneLength >> 8;
      break;
    // B-Dur
    case 'B':   
      ICR1 = frequency_toneB;
      OCR1A = ICR1 >> 1;
      timeToPlay = 1254 * toneLength >> 8;
      break;
    // O-Dur
    case 'O':   
      ICR1 = frequency_toneO;
      OCR1A = 0;  // muted
      timeToPlay = 512 * toneLength >> 8;
      break;
    //When a not allowed Note is given -> no Ton will be played
    default :   
      OCR1A = 0;
      ICR1 = 1;
      timeToPlay = 0;
      break;
    } 
    flag_toneIsPlaying = 1;
}



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

// Set Function: play specific status_system Melody
void set_playSystemMelody(int status_system) {
  switch(status_system) {
    // status_system = System deactivated
    case 0:   
      playTone(tones_statusSystem0[i_tone], toneDurations_statusSystem0[i_tone]);     // play all tones of specific melody after another with defined duration of each tone
      break;
    // status_system = System activated
    case 1:   
      playTone(tones_statusSystem1[i_tone], toneDurations_statusSystem1[i_tone]);
      break;    
    }
    i_tone++;
  }


// Set Function: play systemError Melody
void set_playSystemErrorMelody() {
  playTone(tones_systemError[i_tone], toneDurations_systemError[i_tone]);
  i_tone++;
}


// Set Function: play specific status_drive Melody
void set_playDriveMelody(int status_drive) {        
  switch(status_drive) {
    // status_drive = parking mode
    case 0:   
      playTone(tones_statusDrive0[i_tone], toneDurations_statusDrive0[i_tone]);     // play all tones of specific melody after another with defined duration of each tone
      break;
    // status_drive = sale forward mode
    case 1:   
      playTone(tones_statusDrive1[i_tone], toneDurations_statusDrive1[i_tone]);
      break;
    // status_drive = sale backward mode
    case 2:   
      playTone(tones_statusDrive2[i_tone], toneDurations_statusDrive2[i_tone]);
      break;
    // status_drive = drive forward mode
    case 3:   
      playTone(tones_statusDrive3[i_tone], toneDurations_statusDrive3[i_tone]);
      break;
    // status_drive = recuperation forward mode
    case 4:   
      playTone(tones_statusDrive4[i_tone], toneDurations_statusDrive4[i_tone]);
      break;
    // status_drive = recuperaton backward mode
    case 5:   
      playTone(tones_statusDrive5[i_tone], toneDurations_statusDrive4[i_tone]);
      break;
    // status_drive = drive in reverse mode
    case 6:   
      playTone(tones_statusDrive6[i_tone], toneDurations_statusDrive4[i_tone]);
      break;
    }
    i_tone++;
  }


// Get Function: Curenlty played tone of melody
int get_currentPlayedTone() {
  
  // if the current played tone of melody is before 8 (last tone) -> return the current played tone number
  if (i_tone < numberTones) {
    return i_tone;
  }
  
  // else -> set back to 0 and return 8 (last playable tone number)
  else {
    i_tone = 0;
    return numberTones;
  }
}

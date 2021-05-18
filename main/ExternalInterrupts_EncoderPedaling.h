#ifndef EXTERNAL_INTERRUPT_ENCODER_PEDALING_H     // if not EXTERNAL_INTERRUPT_ENCODER_PEDALING_H (identifier was defined) -> true
#define EXTERNAL_INTERRUPT_ENCODER_PEDALING_H


////// INITIALISATION FUNCTIONS //////

void Init_ExternalInterrupts_EncoderPedaling(void);



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

int get_pedalingRotationalSpeed(long int time_now);

int get_statusPedaling(void);



////// VALIDATION FUNCTIONS //////

int get_pedalingEncoderChannelA(void);

int get_pedalingEncoderChannelB(void);


#endif

#ifndef PWMM_BUZZER_SOUND_H     // if not PWM_BUZZER_SOUND_H (identifier was defined) -> true
#define PWMM_BUZZER_SOUND_H


////// INITIALISATION FUNCTIONS //////

void Init_PWM_BuzzerSound(void);



////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

void set_playSystemMelody(int status_system);

void set_playSystemErrorMelody();

void set_playDriveMelody(int status_drive);

int get_currentPlayedTone();


#endif

#include "TimerMain.h"
#include "ExternalInterrupt_ButtonSystem.h"
#include "ExternalInterrupts_EncoderPedaling.h"
#include "ExternalInterrupt_ReedVelocity.h"
#include "PWM_BuzzerSound.h"
#include "ADC_MotorGeneratorThrottle.h"
#include "Calc_TransformAdcValues.h"
#include "Calc_MotorGeneratorPower.h"
#include "PWM_PedallingWithstand.h"
#include "PWM_motorPowerControlling.h"
#include "Valid_LedRuntime.h"


////// INITIALISATION AND SETUP //////

//// Variables ////
// static = just visible in this software module, volatile = always existing / available for the ISR

// Determining Processing times
static long int time_now = 0.0;     // current global time [nsec]
long int i_timestepMain = 0;        // Counter that counts up the loop cycle of the main function - TimerMain [-]

// TimerMain
static int flag_overload = 0;       // indicates whether Harware was able to make all calculations in cycle Time of TimerMain [-]

// ExternalInterrupt_ButtonSystem - status_system; status_systemError
static int flag_firstStatusButtonInterrupt = 0;   // Flag which avoids to perform the procedure after first Interrupt while Launching the System [-]
static int status_system = 0;                     // Status System: 0 = shutdown, 1 = launched [-]
static int flag_buttonSystemPressed = 0;          // VALIDATION: Flag to indicate whether the Interrupt was executed [-]
static int status_buttonSystemPressed = 0;        // VALIDATION: Contains the status of the Pin - 5V = HIGH / 0V = LOW [-]
static int status_systemError = 0;                // Flag to indicate whether the System Button was pressed while driving - status_drive != 0 [-]
static int timestepSystemButtonPressed = 0;       // VALIDATION: Counter for the timesteps since the System Button is pressed [-]

// ExternalInterrupts_EncoderPedaling - Pedaling rotational Speed; status_pedaling
static signed int pedaling_rotational_speed = 0;  // average rotational speed of last 1/4 rotation [1/min]
static int status_pedaling = 0;                   // Status peddaling mode: 0 = no pedaling, 1 = forward padeling, 2 = backward padeling [-]
static int pedaling_encoderChannelA = 0;          // VALIDATION: Status of Encoder Channel A: 0 = LOW/0V;, 1 = High/5V [-]
static int pedaling_encoderChannelB = 0;          // VALIDATION: Status of Encoder Channel B: 0 = LOW/0V;, 1 = High/5V [-]

// ExternalInterrupt_ReedVelocity - vehicleVelocity; status_drive
static int totalNumberSignals = 0;          // total number how often the button was pushed - variable for function to check the functionality of the code [-]
static int status_drive = 0;                // Status drive mode: 0 = Parking; 1 = sale forward; 2 = sale backward; 3 = Forward drive; 4 = Recuperation forward; 5 = Recuperation backward; 6 = Reverse drive [-]
static double vehicleVelocity = 0.0;        // average vehicle velocity during last wheel rotation [m/s]      // TODO transform to int with a.e. * 100 to int (usage of int to save ressources)
static int timestepSameDriveStatus = 0;     // VALIDATION: timestep counter of current drive_status [-]

// PWM_BuzzerSound - melodies (Sound Generator)
static int numberTones = 8;                 // max number of tones of each melody to play [-]
static int status_systemOld = 0;            // Status System of the last cycle to check whether a melody has to be played [-]
static int flag_statusSystemChange = 0;     // Flag which indicates whether there was a change in the status_system during the last cycle [-]
static int flag_systemError = 0;            // Flag to indicate whether the System Button was pressed while driving - status_drive != 0 [-]
static int status_driveOld = 0;             // Status Drive of the last cycle to check whether a melody has to be played [-]
static int flag_statusDriveChange;          // Flag which indicates whether there was a change in the status_drive during the last cycle [-]

// ADC_MotorGeneratorThrottle - 10BitVariables of voltage and current
#define numberAdcChannels 7                                                   // Number of read ADC Channels [-]
static int adConversionResults[numberAdcChannels] = {0, 1, 2, 3, 4, 5, 6};    // Array which stores the results of the AD converter (ADC0 .. ADC6) [0 ... 1023] [-]
static int flag_adConversionCompleted = 0;                                    // Flag which indicates whether the all ADC values were read / converted in this timestep [-]

static int adc10Bit_potiGenerator = 0;     // ACD0 voltage value [0 mV ... 5000 mV] -> saved as 10 Bit Variable [0 ... 1023] which describes the Potentiometer Voltage of the Generator - unitOfVariable_describedUnitAndSource [-]
static int adc10Bit_potiMotor = 0;         // ACD1 voltage value [0 mV ... 5000 mV] -> saved as 10 Bit Variable [0 ... 1023] which describes the Potentiometer Voltage of the Motor - unitOfVariable_describedUnitAndSource [-] 
static int adc10Bit_potiThrottle = 0;      // ACD2 voltage value [0 mV ... 5000 mV] -> saved as 10 Bit Variable [0 ... 1023] which describes the Potentiometer Voltage of the Throttle - unitOfVariable_describedUnitAndSource [-]

static int adc10Bit_voltageInputGenerator = 0;    // ACD3 voltage value [0 mV ... 5000 mV] -> saved as 10 Bit Variable [0 ... 1023] which describes the Input Voltage of the Generator - unitOfVariable_describedUnitAndSource [-]
static int adc10Bit_currentInputGenerator = 0;    // ACD4 voltage value [0 mV ... 5000 mV] -> saved as 10 Bit Variable [0 ... 1023] which describes the Input Current of the Generator - unitOfVariable_describedUnitAndSource [-]

static int adc10Bit_voltageOutputMotor = 0;       // ACD5 voltage value [0 mV ... 5000 mV] -> saved as 10 Bit Variable [0 ... 1023] which describes the Output Voltage of the Motor - unitOfVariable_describedUnitAndSource [-]
static int adc10Bit_currentOutputMotor = 0;       // ACD6 voltage value [0 mV ... 5000 mV] -> saved as 10 Bit Variable [0 ... 1023] which describes the Output Current of the Motor - unitOfVariable_describedUnitAndSource [-]

// Calc_TransformAdcValues - transformation 10BitVariables into voltages and currents    
static int adcVoltage_potiGenerator = 0;          // Poti Generator: voltage Value [0 mV ... 5000 mV] -> saved as voltage Variable [0 mV ... 5000 mV] - unitOfVariable_describedUnitAndSource [mV]
static int adcPower_potiMotor = 0;                // Poti Motor: voltage Value [0 mV ... 5000 mV] -> saved as Power Variable [50 W ... 320 W] - unitOfVariable_describedUnitAndSource [W]
static double adcPercentage_potiThrottle = 0.0;   // Poti Throttle: voltage Value [0 mV ... 5000 mV] -> saved as Percentage Variable [0 ... 1] - unitOfVariable_describedUnitAndSource [-]

static unsigned int adcVoltage_voltageInputGenerator = 0;     // Generator Input (Generator -> Accumulator): voltage Value [0 mV ... 5000 mV] -> saved as voltage Variable [0 mV ... 50000 mV] - unitOfVariable_describedUnitAndSource [mV]
static signed int adcCurrent_currentInputGenerator = 0;       // Generator Input (Generator -> Accumulator): voltage Value [0 mV ... 5000 mV] -> saved as current Variable [-30000 mA .. 30000 mA] - unitOfVariable_describedUnitAndSource [mA]

static unsigned int adcVoltage_voltageOutputMotor = 0;        // Motor Output (Accumulator -> Motor): voltage Value [0 mV ... 5000 mV] -> saved as voltage Variable [0 mV ... 50000 mV] - unitOfVariable_describedUnitAndSource [mV]
static signed int adcCurrent_currentOutputMotor = 0;          // Motor Output (Accumulator -> Motor): voltage Value [0 mV ... 5000 mV] -> saved as current Variable [-30000 mA .. 30000 mA] - unitOfVariable_describedUnitAndSource [mA]

// PWM_PedallingWithstand
static long int powerActual_generator = 0;    // Actual Power what the Generator currently produces [W]
static double OCR4A_pwmValue = 0;             // OCR4A / on_pwmDutyCycle to adjust the voltage at the PWM Pin6 / PH3 = Pedalling Withstand - PWM voltage between [0 mV ... 5000 mV] [-]

// PWM_MotorPowerControlling
static double factor_linearAdjustment = 0.0;  // Factor to adjust the Power Output of the motor, depening by status_drive and vehicleVelocity -> (0 ... 1) [-]
static double powerTarget_motor = 0;          // Target Power what the Motor should use [W] - depending by status_drive, adcPercentage_potiThrottle, vehicleVelocity -> factor_linearAdjustment [W]
static double powerActual_motor = 0;          // Actual Power what the Motor currently uses [W]
static double powerSmoothed_motor = 0;        // Smoothed Target Power what the Motor should use [W] - smoothed to not have to drastic changes, swings and overshoots [W]
static double OCR4B_pwmValue = 0;             // OCR4B / on_pwmDutyCycle to adjust the voltage at the PWM Pin7 / PH4 = MotorPower - PWM voltage between [1500 mV ... 4500 mV] [-]

// Visualisation in Serial Monitor
static int   i_timestepsToPrint;    // Counter which is used to print the required informations via the serial connection each second [-]

// Validation
static long int time_lastCycle = 0.0;           // VALIDATION: last cylce time [nsec]
static long int time_cycleTime = 0.0;           // VALIDATION: current cycle time [nsec]

static int i_timeAxis = 0;                      // support counter of the x-axis while plotting for validation [unit TimerMain]
static double valueTimeAxis = 0;                // value of the x-axis at specific counter values to visualise time characteristics [-]

static long int threshold_runtime = 10000;      // Threshold of the runtime limit what gets investigated [nsec] - ENTER HERE THRESHOLD OF RUNTIME TO INVESTIGATE [nsec]
static int flag_thresholdRuntimeExceeded = 0;   // Flag which indicates whether the threshold of runtime was exceeded - LED lights up when its extended [-]



//// System Elements ////

void setup() {
  Serial.println("----- Setup main started -----");

  // Initialise Serial Interface to connected PC
  Serial.begin(115200);         // baud rate (speed = [bit/sec])
  delay(100);                   // delay in [ms]
  Serial.println("      setup Serial Interface succesfully finished");
  
  // Configuration modulised Elements
  cli();    //deactivate all interrupts

  Init_TimerMain();
  Serial.println("      setup TimerMain succesfully finished");
  Init_ExternalInterrupt_ButtonSystem();
  Serial.println("      setup ExternalInterrupt_ButtonSystem succesfully finished");
  Init_ExternalInterrupts_EncoderPedaling();
  Serial.println("      setup ExternalInterrupt_EncoderPedaling succesfully finished");
  Init_ExternalInterrupt_ReedVelocity();
  Serial.println("      setup ExternalInterrupt_VehicleVelocity succesfully finished");
  Init_PWM_BuzzerSound();
  Serial.println("      setup PWM_BuzzerSound succesfully finished");
  Init_ADC_MotorGeneratorThrottle();
  Serial.println("      setup ADC_MotorGeneratorThrottle succesfully finished");
  Init_PWM_PedalingWithstand_MotorPower();
  Serial.println("      setup PWM_PedalingWithstand_MotorPower succesfully finished");

  sei();    // Activate all Interrupts
  Serial.println("----- Setup main succesfully finished -----");
}



////// WHILE 1 LOOP MAIN //////

void loop() {   // While(1) loop
  
  //// Time Handling ////
  flag_firstStatusButtonInterrupt = time_now;
  time_now = micros();      // current system time
  i_timestepMain++;         // Validation Variable - COMMENT DURING NORMAL RUN
  i_timestepsToPrint++;
  set_startAdConversion();  // Start AD conversion of the current timestep (as seen in block "ADC_MotorGeneratorThrottle")


    
  //// ExternalInterrupt_ButtonSystem - turn the system on / off ////

  // Determination Application Values
  status_system = get_statusSystem(flag_firstStatusButtonInterrupt, status_system, status_drive);
  status_systemError = get_flagErrorStatusSystem();
    
  // Validation Functions - COMMENT DURING NORMAL RUN
  /*timestepSystemButtonPressed = get_timestepButtonPressed();          
  flag_buttonSystemPressed =  get_flagButtonSystemPressed();
  status_buttonSystemPressed =  get_statusButtonSystemPressed();*/
    

  
  //// ExternalInterrupts_EncoderPedaling - pedaling_rotational_speed; status_pedaling ////

  // Determination Application Values
  pedaling_rotational_speed = get_pedalingRotationalSpeed(time_now);
  status_pedaling = get_statusPedaling();

  // Validation Functions - COMMENT DURING NORMAL RUN
  /*pedaling_encoderChannelA = get_pedalingEncoderChannelA();
  pedaling_encoderChannelB = get_pedalingEncoderChannelB();*/



  //// ExternalInterrupt_ReedVelocity: rear wheel - vehicleVelocity; status_drive ////

  // Determination Application Values
  vehicleVelocity = get_vehicleVelocity(time_now);
  status_drive = get_driveStatus(status_system, status_pedaling, vehicleVelocity);
  timestepSameDriveStatus = get_timestepSameDriveStatus();

  // Validation Functions - COMMENT DURING NORMAL RUN
  /*totalNumberSignals = get_totalNumberSignals();*/


    
  //// PWM_BuzzerSound: Sound Generator ////

  // If the Status of the System has changed since the last timestep or since the melody hasnt played completly -> play melody for changed status_system
  if (status_system != status_systemOld || flag_statusSystemChange == 1) {      
    flag_statusSystemChange = 1;
    set_playSystemMelody(status_system);
  }

  // If a System Error is existing (pushing System Button while driving) or since the melody hasnt played completly -> play melody for System Error
  if (status_systemError == 1 || flag_systemError == 1) {
    flag_systemError = 1;
    set_playSystemErrorMelody();   
  }
    
  // If the Status of Drive has changed since the last timestep or since the melody hasnt played completly -> play melody for changed status_drive
  if (status_drive != status_driveOld || flag_statusDriveChange == 1) { 
    flag_statusDriveChange = 1;     
    set_playDriveMelody(status_drive);
  }

  // If the certain melody has been played completly -> set back the flags for changed status_system and status_drive
  if (get_currentPlayedTone() == numberTones) {                            
    flag_statusSystemChange = 0;
    flag_systemError = 0;
    flag_statusDriveChange = 0;
  }

  // Prepare Variables for the next cycle
  status_systemOld = status_system;
  status_driveOld = status_drive;
 


  //// ADC_MotorGeneratorThrottle ////

  // Convert ADC Channels
  while (get_flagAdConversionCompleted() == 0) {}   // Wait until conversion of all AD Channels is finished (process was started in Block "Time Handling")
   
  // Read converted values of AD Channels
  memcpy(adConversionResults, get_adConversionResults(), sizeof(adConversionResults));    // TODO Function is very time consuming and should be replaced
  adc10Bit_potiGenerator = adConversionResults[0];     // ADC0 = Pin A0
  adc10Bit_potiMotor = adConversionResults[1];         // ADC1 = Pin A1
  adc10Bit_potiThrottle = adConversionResults[2];      // ADC2 = Pin A2
  
  adc10Bit_voltageInputGenerator = adConversionResults[3];    // ADC3 = Pin A3
  adc10Bit_currentInputGenerator = adConversionResults[4];    // ADC4 = Pin A4
    
  adc10Bit_voltageOutputMotor = adConversionResults[5];       // ADC5 = Pin A5
  adc10Bit_currentOutputMotor = adConversionResults[6];       // ADC6 = Pin A6



  //// Calc_TransformAdcValues - Calculations Voltage, Current, Power ////

  // Poti Generator
  adcVoltage_potiGenerator = get_10BitTransformedTo5000mV(adc10Bit_potiGenerator);   // 10bit -> voltage [0 mV .. 5000 mV]

  // Poti Motor
  adcPower_potiMotor = get_10BitTransformedTo270W(adc10Bit_potiMotor);               // 10bit -> power [50 W .. 320 W]
    
  // Poti Throttle
  adcPercentage_potiThrottle = get_10BitTransformedTo1(adc10Bit_potiThrottle);       // 10bit -> percentage [0 .. 1]

  // Generator Input (Generator -> Accumulator)
  adcVoltage_voltageInputGenerator = get_10BitTransformedTo50000mV(adc10Bit_voltageInputGenerator);    // 10bit -> voltage [0 mV .. 50000 mV]
  adcCurrent_currentInputGenerator = get_10BitTransformedTo60000mA(adc10Bit_currentInputGenerator);    // 10bit -> current [-30000 mA .. 30000 mA]

  // Motor Output (Accumulator -> Motor)
  adcVoltage_voltageOutputMotor = get_10BitTransformedTo50000mV(adc10Bit_voltageOutputMotor);          // 10bit -> voltage [0 mV .. 50000 mV]
  adcCurrent_currentOutputMotor = get_10BitTransformedTo60000mA(adc10Bit_currentOutputMotor);          // 10bit -> current [-30000 mA .. 30000 mA]
        
    

  //// PWM_PedallingWithstand ////
     
  // Determine the power_generator value
  powerActual_generator = get_powerOutOfVoltageAndCurrent(adcVoltage_voltageInputGenerator, adcCurrent_currentInputGenerator);
    
  // calculate the current PWM Pedalling Withstand value between [0 mV ... 5000 mV]
  OCR4A_pwmValue = get_generatorPwmControlValue(adcVoltage_potiGenerator, adcVoltage_voltageInputGenerator, adcVoltage_voltageOutputMotor);
  OCR4A = OCR4A_pwmValue;   // OCR4A = Register on Microcontroller to adjust PWM voltage on Pin6 / PH3



  //// PWM_MotorPowerControlling ////
    
  // Determine the power_motor value to control 
  powerActual_motor = get_powerOutOfVoltageAndCurrent(adcVoltage_voltageOutputMotor, adcCurrent_currentOutputMotor);            // determina the actual power_motor

  factor_linearAdjustment = get_linearMotorPowerAdjustment(status_drive, vehicleVelocity);                                                  // determine the linear adjustment depending by status_drive and vehicleVelocity
  powerTarget_motor = get_powerOutOfMaxValueAndThrottlePosition(adcPower_potiMotor, adcPercentage_potiThrottle, factor_linearAdjustment);   // determina the target power_motor
    
  powerSmoothed_motor = get_PiControllerValue(powerActual_motor, powerTarget_motor);               // determine a smoothed power_motor to control with a PID Controller

  // calculate the current PWM throttle value between [1500 mV ... 4500 mV]
  OCR4B_pwmValue = get_motorPwmControlValue(powerSmoothed_motor);        // OCR4B value to adjust the Motor Power (voltage at the specific PWM pin) - PWM voltage between [1500 mV ... 4500 mV]
  OCR4B = OCR4B_pwmValue;   // OCR4B = Register on Microcontroller to adjust PWM voltage on Pin7 / PH4
    


  //// Visualisation in Serial Monitor ////

  // If 1 sec have passed -> print
  if (i_timestepsToPrint == 100) {
      
    // Plot in Serial Monitor: ExternalInterrupt_EncoderPedals & ExternalInterrupt_ReedVelocity - all
    Serial.println("tTimestep\Pedals_RotSpeed\tVehVelocity\tGen_ActPower\tMot_ActPower\tGenPoti_voltage\tMotPoti_Power\tThrPoti_Percentage");   // print header
    Serial.print(i_timestepMain);
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(pedaling_rotational_speed);
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(vehicleVelocity);
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(powerActual_generator);
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(powerActual_motor);
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(adcVoltage_potiGenerator);
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(adcPower_potiMotor);
    Serial.print("\t");
    Serial.print("\t");
    Serial.println(adcPercentage_potiThrottle);

    i_timestepsToPrint = 0;
  }



  //// Validation and Visualisation of the System - COMMENT DURING NORMAL RUN, UNCOMMENT JUST 1 VALIDATION PLOT BLOCK DURING VALDIATION RUN ////
    
  // Timer_MainLoop: Check whether it is still running - calculation time is within Timer time
  /*if (get_TimerMain_timestepCompleted() == 0) {
    flag_overload = 0;                    // Timer ist still runing = no overload (all good)
  } 
  else {
    flag_overload = 1;                    // Timmer reached time limit already = Hardware is overloaded
  }*/

  // Determine and plot calculation time last cycle
  /*time_cycleTime = time_now - time_lastCycle;
  time_lastCycle = time_now;
  flag_thresholdRuntimeExceeded = get_thresholdRuntimeExceeded(threshold_runtime, time_cycleTime);*/


  // Plot in Serial Monitor: Time Handling
  /*Serial.println("NumberSignals\tThresholdRuntime\tCycleTime\tRuntimeExceeded\tOverload");   // print header
  Serial.print(i_timestepMain);                              
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(threshold_runtime);                                 
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(time_cycleTime);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(flag_thresholdRuntimeExceeded);      
  Serial.print("\t");
  Serial.print("\t");
  Serial.println(flag_overload);*/    


  // Plot in Serial Monitor: ExternalInterrupt_VehicleVelocity - all
  /*Serial.println("NumberSignals\tVelocity\tOverload\tCycleTime");   // print header
  Serial.print(totalNumberSignals);                                 // print calculated vehicle velocity
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(vehicleVelocity);                                   // print calculated vehicle velocity
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(flag_overload);                                      // print status of flag_overload
  Serial.print("\t");
  Serial.print("\t");
  Serial.println(time_cycleTime);                                   // print cycle time of all calculations*/


  // Plot in Serial Monitor: ExternalInterrupt_EncoderPedals & ExternalInterrupt_ReedVelocity - all status and their timesteps
  /*Serial.println("\TimestepButtonPressed\tStatusSystem\tErrorSystem\tRotationalSpeed\tPedallingStatus\tChannelA\tChannelB\tTimestepsCurrentDrive\tStatusDrive\tVelocity\tCurrentTone\tOverload\tCycleTime");   // print header
  Serial.print(timestepSystemButtonPressed);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(status_system);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(status_systemError);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(pedaling_rotational_speed);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(status_pedaling);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(pedaling_encoderChannelA);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(pedaling_encoderChannelB);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(timestepSameDriveStatus);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(status_drive);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(vehicleVelocity);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(get_currentPlayedTone());
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(flag_overload);
  Serial.print("\t");
  Serial.print("\t");
  Serial.println(time_cycleTime);*/

  
  // Plot in Serial Monitor: ExternalInterrupt_EncoderPedals & ExternalInterrupt_ReedVelocity - all status and flags
  /*Serial.println("\TimestepButtonPressed\tFlagButton\tStatusButton\tStatusSystem\tErrorSystem\tRotationalSpeed\tPedallingStatus\tChannelA\tChannelB\tNumberReedSignals\tTimestepsCurrentDrive\tStatusDrive\tVelocity\tOverload\tCycleTime");   // print header
  Serial.print(timestepSystemButtonPressed);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(flag_buttonSystemPressed);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(status_buttonSystemPressed);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(status_system);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(status_systemError);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(pedaling_rotational_speed);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(status_pedaling);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(pedaling_encoderChannelA);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(pedaling_encoderChannelB);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(totalNumberSignals);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(timestepSameDriveStatus);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(status_drive);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(vehicleVelocity);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(flag_overload);
  Serial.print("\t");
  Serial.print("\t");
  Serial.println(time_cycleTime);*/


  // Plot in Serial Monitor: ADC_MotorGeneratorThrottle - all
  /*Serial.println("\A0_PotiGenerator\tA1-PotiMotor\tA2-PotiThrottle\tA3-VoltageGenerator\tA4-CurrentGenerator\tA5-VoltageMotor\tA6-CurrentMotor");   // print header
  Serial.print(adc10Bit_potiGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_potiMotor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_potiThrottle);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_voltageInputGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_currentInputGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_voltageOutputMotor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.println(adc10Bit_currentOutputMotor);*/
    

  // Plot in Serial Monitor: ADC_MotorGeneratorThrottle & Calc_TransformAdcValues - all
  /*Serial.println("1024-PotiGen\t5000mV-PotiGen\t1024-PotiMot\t320W-PotiMot\t1024-PotiThr\t1-PotiThr\t1024-VoltGen\t50000mV-VoltGen\t1024-CurGen\t30000mA-CurGen\t1024-VoltMot\t50000mV-VoltMot\t1024-CurMot\t30000mA-CurMot");   // print header
  Serial.print(adc10Bit_potiGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adcVoltage_potiGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_potiMotor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adcPower_potiMotor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_potiThrottle);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adcPercentage_potiThrottle);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_voltageInputGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adcVoltage_voltageInputGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_currentInputGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adcCurrent_currentInputGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_voltageOutputMotor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adcVoltage_voltageOutputMotor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adc10Bit_currentOutputMotor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.println(adcCurrent_currentOutputMotor);*/


  // Plot in Serial Monitor: Calc_TransformAdcValues & PWM_PedallingWithstand - all
  /*Serial.println("t5000mV-PotiGen\tActPow_Gen\tOCR4A_Gen\t50000mV-VoltGen\t30000mA-CurGen");   // print header
  Serial.print(adcVoltage_potiGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(powerActual_generator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(OCR4A);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adcVoltage_voltageInputGenerator);
  Serial.print("\t");
  Serial.print("\t");
  Serial.println(adcCurrent_currentInputGenerator);*/


  // Plot in Serial Monitor: Calc_TransformAdcValues & PWM_MotorPowerControlling - all
  /*Serial.println("t50000mV-VoltMot\t30000mA-CurMot\tActPow_Mot\t320W-PotiMot\t1-PotiThr\tAdjustment\tTarPow_Mot\tSmoPow_Mot\tOCR4B_Mot");   // print header
  Serial.print(adcVoltage_voltageOutputMotor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adcCurrent_currentOutputMotor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(powerActual_motor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adcPower_potiMotor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(adcPercentage_potiThrottle);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(factor_linearAdjustment);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(powerTarget_motor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(powerSmoothed_motor);
  Serial.print("\t");
  Serial.print("\t");
  Serial.println(OCR4B_pwmValue);*/


  // Plot in Serial Plotter: ExternalInterrupts_EncoderPedaling - status_pedaling
  /*i_timeAxis++;
  Serial.println("Time\tEncoderChannelA\tEncoderChannelB\tPedallingStatus");  // header

  // If 1 sec has passed (happens 1 time in lifetime of counter - 1 sec) -> print 1 sec mark
  if (i_timeAxis == 100) {           // if i_timeAxis == 100
    valueTimeAxis = -0.5;
  }
  // If 0.5 sec has passed (happens 1 time in lifetime of counter - 1 sec) -> print 0.5 sec mark
  else if (i_timeAxis == 50) {       // if i_timeAxis == 50
    valueTimeAxis = -1;
  }
  // If 0.1 sec has passed (happens 8 times in lifetime of counter - 1 sec) -> print 0.1 sec mark
  else if (i_timeAxis % 10 == 0) {   // if i_timeAxis == 10, 20, 30, 40, 60, 70, 80, 90
    valueTimeAxis = -1.5;
  }
  // Else -> print no mark
  else {
    valueTimeAxis = -2;
  }
  // If lifetime of counter is reached (1 sec) -> set back counter to 1
  if (i_timeAxis > 100) {
    i_timeAxis = 1;
  }
  
  Serial.print(valueTimeAxis);
  Serial.print(" ");
  Serial.print(pedaling_encoderChannelA);
  Serial.print(" ");
  Serial.print(pedaling_encoderChannelB);
  Serial.print(" ");
  Serial.print(status_pedaling);
  Serial.println(" ");*/

   
  // Plot in Serial Plotter: ExternalInterrupts_EncoderPedaling & ExternalInterrupt_ReedVelocity - status_pedaling; status_drive
  /*i_timeAxis++;
  Serial.println("Time\tVehicleVelocity\tEncoderChannelA\tEncoderChannelB\tDriveStatus\tPedallingStatus");  // header
  
  // If 1 sec has passed (happens 1 time in lifetime of counter - 1 sec) -> print 1 sec mark
  if (i_timeAxis == 100) {           // if i_timeAxis == 100
    valueTimeAxis = -0.5;
  }
  // If 0.5 sec has passed (happens 1 time in lifetime of counter - 1 sec) -> print 0.5 sec mark
  else if (i_timeAxis == 50) {       // if i_timeAxis == 50
    valueTimeAxis = -1;
  }
  // If 0.1 sec has passed (happens 8 times in lifetime of counter - 1 sec) -> print 0.1 sec mark
  else if (i_timeAxis % 10 == 0) {   // if i_timeAxis == 10, 20, 30, 40, 60, 70, 80, 90
    valueTimeAxis = -1.5;
  }
  // Else -> print no mark
  else {
    valueTimeAxis = -2;
  }
  // If lifetime of counter is reached (1 sec) -> set back counter to 1
  if (i_timeAxis > 100) {
    i_timeAxis = 1;
  }
  
  Serial.print(valueTimeAxis);
  Serial.print(" ");
  Serial.print(vehicleVelocity);
  Serial.print(" ");
  Serial.print(pedaling_encoderChannelA);
  Serial.print(" ");
  Serial.print(pedaling_encoderChannelB);
  Serial.print(" ");
  Serial.print(status_drive);
  Serial.print(" ");
  Serial.print(status_pedaling);
  Serial.println(" ");*/
    


  //// Cycle Time Handling Timer_MainLoop ////

  // Fill up remaining time
  while (get_flagTimestepCompleted() == 0) {    // be in while loop as long as Timer 3 hasnt reached its time limit
  }     

  // Set back flag = cycle time fullfilled
  set_flagNewTimestep();
    
}

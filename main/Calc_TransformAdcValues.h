#ifndef CALC_TRANSFORM_ADC_VALUES_H     // if not CALC_TRANSFORM_ADC_VALUES_H (identifier was defined) -> true
#define CALC_TRANSFORM_ADC_VALUES_H 


////// GET (READ) / SET (WRITE) FUNCTIONS ////// 

//// Voltage ////

int get_10BitTransformedTo3000mV(int adc10BitDigital);

int get_10BitTransformedTo5000mV(int adc10BitDigital);

int get_10BitTransformedTo50000mV(int adc10BitDigital);



//// Percentage ////

double get_10BitTransformedTo1(int adc10BitDigital);



//// Power ////

int get_10BitTransformedTo270W(int adc10BitDigital);



//// Current ////

signed int get_10BitTransformedTo60000mA(int adc10BitDigital);


#endif

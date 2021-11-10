#ifndef CALIBRATION_H
#define CALIBRATION_H

/* 
Two calibration values are used in this sketch: powerCal, and phaseCal.
With most hardware, the default values are likely to work fine without
need for change.  A compact explanation of each of these values now follows:

When calculating real power, which is what this code does, the individual
conversion rates for voltage and current are not of importance.  It is
only the conversion rate for POWER which is important.  This is the
product of the individual conversion rates for voltage and current.  It
therefore has the units of ADC-steps squared per Watt.  Most systems will
have a power conversion rate of around 20 (ADC-steps squared per Watt).

powerCal is the RECIPR0CAL of the power conversion rate.  A good value
to start with is therefore 1/20 = 0.05 (Watts per ADC-step squared)

## Voltage calibration constant:
AC-AC Voltage adapter is designed to step down the voltage from 230V to 9V
but the AC Voltage adapter is running open circuit and so output voltage is
likely to be 20% higher than 9V (9 x 1.2) = 10.8V.
The output voltage is then steped down further with the voltage divider which has
values Rb = 10k, Rt = 120k (which will reduce the voltage by 13 times).

NZ calculations: VAC = 240
Output voltage of AC-AC adapter = 11.23
EmonTx shield voltage divider = 11
  --> ( 240 / 11.23 ) * 11 = 235.1

## Current calibration constant:
CT ratio / burden resistance for EmonTX shield
  --> (100A / 0.05A) / 33 Ohms = 60.606

NOTE: ADC ref voltage is 3.3V for emonTx, 5.0V for Arduino

See initialiseCalibration() in main sketch;

  powerCal = (VOLTAGE_CALIBRATION * (ADC_REF_VOLTAGE / ADC_STEPS)) * (CURRENT_CALIBRATION * (ADC_REF_VOLTAGE / ADC_STEPS));

*/
#define       ADC_REF_VOLTAGE           5.0
#define       ADC_STEPS                 1023
#define       CYCLES_PER_SEC            50

#define       VOLTAGE_CALIBRATION       235.1
#define       CURRENT_CALIBRATION       60.606

// NB. Any tool which determines the optimal value of phaseCal must have a similar
// scheme for taking sample values as does this sketch!
// http://openenergymonitor.org/emon/node/3792#comment-18683
const float PHASE_CAL_OFFSET[CT_COUNT]  = { 0.2, 0.4, 0.6, 0.8 };

#endif

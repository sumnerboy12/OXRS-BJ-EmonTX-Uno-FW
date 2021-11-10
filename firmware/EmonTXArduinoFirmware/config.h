#ifndef CONFIG_H
#define CONFIG_H

/* ----------------- General config -------------------------------- */
/* Serial */
#define       SERIAL_BAUD_RATE          115200

/* Ethernet */
#define       DHCP_TIMEOUT_MS           15000
#define       DHCP_RESPONSE_TIMEOUT_MS  4000

// dnsmasq static lease for these MAC addresses (emontx-01/02/03)
const byte    STATIC_MAC[]            = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x3C };
//const byte    STATIC_MAC[]            = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x3D };
//const byte    STATIC_MAC[]            = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x3E };

/* MQTT */
const char *  MQTT_BROKER             = "mqtt.home";
const int     MQTT_PORT               = 1883;
const char *  MQTT_USERNAME           = "emontx";
const char *  MQTT_PASSWORD           = "qQvZnnhJt9bM5g4FUmHr";

/* How many CT clamps being monitored and which one is the grid feed */
#define       CT_COUNT                  4
#define       CT_GRID                   0

/* Data logger */
#define       DATALOG_PERIOD_SECS       5
#define       LED_PULSE_MS              50

/* Diverter */
#define       DEFAULT_DIVERTER_ENABLED  false
#define       DIVERTER_HYSTERISIS       0.1
#define       SWEETZONE_IN_JOULES       3600


/* ------------------------- Calibration --------------------------- */
// Two calibration values are used in this sketch: powerCal, and phaseCal.
// With most hardware, the default values are likely to work fine without
// need for change.  A compact explanation of each of these values now follows:

// When calculating real power, which is what this code does, the individual
// conversion rates for voltage and current are not of importance.  It is
// only the conversion rate for POWER which is important.  This is the
// product of the individual conversion rates for voltage and current.  It
// therefore has the units of ADC-steps squared per Watt.  Most systems will
// have a power conversion rate of around 20 (ADC-steps squared per Watt).

// powerCal is the RECIPR0CAL of the power conversion rate.  A good value
// to start with is therefore 1/20 = 0.05 (Watts per ADC-step squared)

// ## Voltage calibration constant:
// AC-AC Voltage adapter is designed to step down the voltage from 230V to 9V
// but the AC Voltage adapter is running open circuit and so output voltage is
// likely to be 20% higher than 9V (9 x 1.2) = 10.8V.
// The output voltage is then steped down further with the voltage divider which has
// values Rb = 10k, Rt = 120k (which will reduce the voltage by 13 times).

// NZ calculations: VAC = 240
// Output voltage of AC-AC adapter = 11.23
// EmonTx shield voltage divider = 11
// --> ( 240 / 11.23 ) * 11 = 235.1

// ## Current calibration constant:
// CT ratio / burden resistance for EmonTX shield
// --> (100A / 0.05A) / 33 Ohms = 60.606

// NOTE: ADC ref voltage is 3.3V for emonTx, 5.0V for Arduino
#define       ADC_REF_VOLTAGE           5.0
#define       ADC_STEPS                 1023
#define       CYCLES_PER_SECOND         50

#define       VOLTAGE_CALIBRATION       235.1
#define       CURRENT_CALIBRATION       60.606

// Pre-calculated scale factor to get accurate results
// Can be updated via MQTT config
#define       DEFAULT_SCALE_FACTOR      1.17947

#endif

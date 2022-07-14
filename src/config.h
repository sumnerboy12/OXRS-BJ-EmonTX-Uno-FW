#ifndef CONFIG_H
#define CONFIG_H

/* Ethernet MAC address - we add MAC_ADDRESS_OFFSET if defined */
const unsigned char BASE_MAC_ADDRESS[]  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x3C };

/* How many CT clamps being monitored and which one is the grid feed */
#define       CT_COUNT                    4
#define       CT_GRID                     0

/* Data logger */
#define       DATALOG_PERIOD_SECS         5
#define       LED_PULSE_MS                50

/* Calibration */
#define       DEFAULT_SCALE_FACTOR        1.17947

/* Diverter */
#define       DEFAULT_DIVERTER_ENABLED    false
#define       SWEETZONE_IN_JOULES         3600

#endif

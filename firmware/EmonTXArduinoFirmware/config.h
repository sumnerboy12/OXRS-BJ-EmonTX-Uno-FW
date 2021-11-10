#ifndef CONFIG_H
#define CONFIG_H

/* Ethernet */
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

/* Calibration */
#define       DEFAULT_SCALE_FACTOR      1.17947

/* Diverter */
#define       DEFAULT_DIVERTER_ENABLED  false
#define       DIVERTER_HYSTERISIS       0.1
#define       SWEETZONE_IN_JOULES       3600

#endif

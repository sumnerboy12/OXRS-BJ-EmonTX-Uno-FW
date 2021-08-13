#ifndef CONFIG_H
#define CONFIG_H

/* ----------------- General config -------------------------------- */
/* Serial */
#define       SERIAL_BAUD_RATE        115200

/* Ethernet */
#define       ENABLE_DHCP             true

// can't use MAC address ROM on EtherTen since the EmonTX shield
// uses ADC0-4 for V and I sensors and the I2C bus uses ADC4-5
const byte    STATIC_MAC[]          = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x3C };
//const byte    STATIC_MAC[]          = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x3D };
//const byte    STATIC_MAC[]          = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x3E };

const byte    STATIC_IP[]           = { 192, 168, 40, 21 };
const byte    STATIC_DNS[]          = { 192, 168, 40, 100 };

/* MQTT */
#define       MQTT_BACKOFF_SECS       5
#define       MQTT_MAX_BACKOFF_COUNT  12
const char *  MQTT_BROKER           = "mqtt.home";
const int     MQTT_PORT             = 1883;
const char *  MQTT_CLIENT_ID        = NULL;
const char *  MQTT_USERNAME         = "emontx";
const char *  MQTT_PASSWORD         = "qQvZnnhJt9bM5g4FUmHr";
const char *  MQTT_BASE_TOPIC       = "energy";

#define       ENABLE_MQTT_LWT         true
const char *  MQTT_LWT_BASE_TOPIC   = "clients";
const uint8_t MQTT_LWT_QOS          = 0;
const uint8_t MQTT_LWT_RETAIN       = 1;

/* ----------------- Hardware-specific config ---------------------- */
/* EmonTX */
#define       DATALOG_PERIOD_SECS     5
#define       CYCLES_PER_SECOND       50

/* Diverter */
#define       ENABLE_DIVERTER         false
#define       SWEETZONE_IN_JOULES     3600

/* Watchdog */
#define       ENABLE_WATCHDOG         true
#define       WATCHDOG_PIN            3
#define       WATCHDOG_PULSE_MS       50
#define       WATCHDOG_RESET_MS       30000

#endif

//      Code by Robin Emley (calypso_rae on Open Energy Monitor Forum) - September 2013
//      Updated November 2013 to include analog and LED pins for the emonTx V3 by Glyn Hudson
//
//      Updated July 2014 to send readings via MQTT by Ben Jones
//      Updated October 2014 to include PV router functionality by Ben Jones
//      Updated January 2015 to add support for remote config of scale factor (by MQTT)
//
//      The interrupt-based kernel for this sketch was kindly provided by Jorg Becker.

/*--------------------------- Version ------------------------------------*/
#define VERSION "1.0"

/*--------------------------- Configuration ------------------------------*/
// Should be no user configuration in this file, everything should be in;
#include "config.h"

/*--------------------------- Libraries ----------------------------------*/
#include <avr/wdt.h>
#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include <PubSubClient.h>

/*--------------------------- Constants ----------------------------------*/
// these values relate to the HIGH/LOW handling in the heating node
#define DIVERT_ON                 1
#define DIVERT_OFF                0
#define LED_ON                    1
#define LED_OFF                   0

/*--------------------------- Global Variables ---------------------------*/
// if no mqtt_client_id set in config.h defaults to "ETX-<MAC ADDRESS>"
char g_mqtt_client_id[16];

// we are sending calculated results to an MQTT topic via ethernet
char g_mqtt_data_topic[32];
char g_mqtt_config_topic[32];
char g_mqtt_divert_topic[32];
char g_mqtt_lwt_topic[32];

// when reconnecting to MQTT broker backoff in increasing intervals
uint8_t g_mqtt_backoff = 0;

// last time we attempted to reconnect to MQTT
uint32_t g_mqtt_last_reconnect_ms = 0L;

// struct to store our readings from each CT
typedef struct {
   int power_CT1;
   int power_CT2;
   int power_CT3;
   int power_CT4; } TX_DATA;
TX_DATA tx_data;

// definition of enumerated types
enum POLARITY           { NEGATIVE, POSITIVE };
enum DIVERTER_MODES     { ANTI_FLICKER, NORMAL, DISABLED };

// The diverter logic can operate in either of these modes:
//    - NORMAL          switch rapidly on/off to maintain a constant energy level
//    - ANTI_FLICKER    reduced switch rate to avoid rapid fluctuations of the local mains voltage
//    - DISABLED        diverter disabled
enum DIVERTER_MODES diverterMode = DISABLED;

// flag for enabling/disabling the diverter
boolean diverterEnabled = false;
int divertCurrent = -1;

// LED pulse to indicate something happening
unsigned long LED_onAt;
int LED_onMs = 50;

/*--------------------------- Pinout assignments -------------------------*/
// digital input pins:
// dig pin 0 is for Serial Rx
// dig pin 1 is for Serial Tx
// dig pin 4 is for the ethernet module (IRQ)
// dig pin 10 is for the ethernet module (SEL)
// dig pin 11 is for the ethernet module (SDI)
// dig pin 12 is for the ethernet module (SDO)
// dig pin 13 is for the ethernet module (CLK)
const byte LED_pin = 9;

// analogue input pins (for emonTx V3):
const byte voltageSensor      = 0;      // analogue
const byte currentSensor_CT1  = 1;      // analogue
const byte currentSensor_CT2  = 2;      // analogue
const byte currentSensor_CT3  = 3;      // analogue
const byte currentSensor_CT4  = 4;      // analogue

/*--------------------------- Energy calculation -------------------------*/
// Some of these variables are used in multiple blocks so cannot be static.
// For integer maths, many variables need to be 'long'
boolean beyondStartUpPhase    = false;  // start-up delay, allows things to settle
const byte startUpPeriod      = 3;      // in seconds, to allow LP filter to settle
const int DCoffset_I          = 512;    // nominal mid-point value of ADC @ x1 scale

long capacityOfEnergyBucket;            // depends on powerCal, frequency & the 'sweetzone' size.
long lowerEnergyThreshold;              // for turning diverter off
long upperEnergyThreshold;              // for turning diverter on

long energyInBucket           = 0;      // to record the present level in the energy accumulator for PV diversion
long energyInBucket_CT1       = 0;      // to record the present level in the energy accumulator for data logging
long energyInBucket_CT2       = 0;      // to record the present level in the energy accumulator for data logging
long energyInBucket_CT3       = 0;      // to record the present level in the energy accumulator for data logging
long energyInBucket_CT4       = 0;      // to record the present level in the energy accumulator for data logging
int phaseCal_CT1;                       // to avoid the need for floating-point maths
int phaseCal_CT2;                       // to avoid the need for floating-point maths
int phaseCal_CT3;                       // to avoid the need for floating-point maths
int phaseCal_CT4;                       // to avoid the need for floating-point maths
long DCoffset_V;                        // <--- for LPF
long DCoffset_V_min;                    // <--- for LPF
long DCoffset_V_max;                    // <--- for LPF

// this setting is only used if anti-flicker mode is enabled (must not exceed 0.5)
const float offsetOfEnergyThresholdsInAFmode = 0.1;

// for interaction between the main processor and the ISR
volatile boolean dataReady = false;
int sampleI_CT1;
int sampleI_CT2;
int sampleI_CT3;
int sampleI_CT4;
int sampleV;

/*--------------------------- Calibration --------------------------------*/
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

// Voltage calibration constant:

// AC-AC Voltage adapter is designed to step down the voltage from 230V to 9V
// but the AC Voltage adapter is running open circuit and so output voltage is
// likely to be 20% higher than 9V (9 x 1.2) = 10.8V.
// Open circuit step down = 230 / 10.8 = 21.3

// The output voltage is then steped down further with the voltage divider which has
// values Rb = 10k, Rt = 120k (which will reduce the voltage by 13 times.

// The combined step down is therefore 21.3 x 13 = 276.9 which is the
// theoretical calibration constant entered below.

// NZ voltage = 240
// Output voltage of AC-AC adapter = 11.23
// EmonTx shield voltage divider = 11
// --> ( 240 / 11.23 ) * 11 = 235.1

// Current calibration constant:

// CT ratio / burden resistance for EmonTX shield
// --> (100A / 0.05A) / 33 Ohms = 60.606

// Measured load using the powerCal below with no scale factor
// - first HWC (3kW)  = 3000 / 1126.9 = 2.66217
// - then Enasolar    = 1.71 / 1.78   = 0.96967 = 2.55748 (instantaneous)
// - then Enasolar    = 1.78 / 1.76   = 1.01136 = 2.58654 (instantaneous)
// - then Enasolar    = 6.47 / 6.70   = 0.96567 = 2.49774 (daily total)
// - then spreadsheet =               = 0.97    = 2.42281 (2014-08-27)
// - then spreadsheet =               = 1.00683 = 2.43936 (2014-09-03)
// - then manual      =               = 1.135   = 2.76867 (2015-01-21 - full month report from openHAB vs Meridian)

// default/startup values
float scaleFactor_CT1 = 2.70769;  // Grid
float scaleFactor_CT2 = 2.70769;  // Solar (Enasolar output)
float scaleFactor_CT3 = 2.70769;  // HWC (3kW)
float scaleFactor_CT4 = 2.70769;  // UFHP

// phaseCal is used to alter the phase of the voltage waveform relative to the
// current waveform.  The algorithm interpolates between the most recent pair
// of voltage samples according to the value of phaseCal.
//
//    With phaseCal = 1, the most recent sample is used.
//    With phaseCal = 0, the previous sample is used
//    With phaseCal = 0.5, the mid-point (average) value in used
//
// NB. Any tool which determines the optimal value of phaseCal must have a similar
// scheme for taking sample values as does this sketch!
// http://openenergymonitor.org/emon/node/3792#comment-18683
const float phaseCalOffset_CT1 = 0.2;
const float phaseCalOffset_CT2 = 0.4;
const float phaseCalOffset_CT3 = 0.6;
const float phaseCalOffset_CT4 = 0.8;

int datalogCountInMainsCycles;
const int maxDatalogCountInMainsCycles = DATALOG_PERIOD_SECS * CYCLES_PER_SECOND;
float normalisation_CT1;
float normalisation_CT2;
float normalisation_CT3;
float normalisation_CT4;

/*--------------------------- Function Signatures ------------------------*/
void mqttCallback(char * topic, byte * payload, int length);

/*--------------------------- Instantiate Global Objects -----------------*/
// ethernet shield uses pins 4 for SD card and 10-13 for SPI
EthernetClient ethernet;

// MQTT client
PubSubClient mqtt_client(MQTT_BROKER, MQTT_PORT, mqttCallback, ethernet);

/*--------------------------- Program ------------------------------------*/
void setup() {
  // ensure the watchdog is disabled
  wdt_disable();

  // initialise the serial interface
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println();
  Serial.println(F("======================="));
  Serial.println(F("    EmonTX Diverter"));
  Serial.print  (F("         v"));
  Serial.println(VERSION);
  Serial.println(F("======================="));

  // setup indicator LED
  pinMode(LED_pin, OUTPUT);

  // start the I2C bus
  Wire.begin();

  // start the SPI bus
  SPI.begin();

  // initialise ethernet/MQTT
  initialiseNetwork();

  // ensure the diverter is OFF
  setDivert(DIVERT_OFF);

  // setup our calibration variables
  initialiseCalibration();

  // initialse the ADC and setup the interrupt handler
  initialiseADC();

  // update all our config parameters, using the default/startup values
  updateCalculationParameters();
  
  // enable the watchdog timer - 8s timeout
  wdt_enable(WDTO_8S);
  wdt_reset();
}

// An Interrupt Service Routine is now defined in which the ADC is instructed to perform
// a conversion of the voltage signal and each of the signals for current.  A "data ready"
// flag is set after each voltage conversion has been completed, it being the last one in
// the sequence.
//   Samples for current are taken first because the phase of the waveform for current is
// generally slightly advanced relative to the waveform for voltage.  The data ready flag
// is cleared within loop().

// This Interrupt Service Routine is for use when the ADC is in the free-running mode.
// It is executed whenever an ADC conversion has finished, approx every 104 us.  In
// free-running mode, the ADC has already started its next conversion by the time that
// the ISR is executed.  The ISR therefore needs to "look ahead".
//   At the end of conversion Type N, conversion Type N+1 will start automatically.  The ISR
// which runs at this point therefore needs to capture the results of conversion Type N ,
// and set up the conditions for conversion Type N+2, and so on.
//
ISR(ADC_vect) {
  static unsigned char sample_index = 0;

  switch (sample_index) {
    case 0:
      sampleV = ADC;
      ADMUX = 0x40 + currentSensor_CT2;     // set up the next-but-one conversion
      sample_index++;                       // advance the control flag
      dataReady = true;
      break;
    case 1:
      sampleI_CT1 = ADC;
      ADMUX = 0x40 + currentSensor_CT3;     // for the next-but-one conversion
      sample_index++;                       // advance the control flag
      break;
    case 2:
      sampleI_CT2 = ADC;
      ADMUX = 0x40 + currentSensor_CT4;     // for the next-but-one conversion
      sample_index++;                       // advance the control flag
      break;
    case 3:
      sampleI_CT3 = ADC;
      ADMUX = 0x40 + voltageSensor;         // for the next-but-one conversion
      sample_index++;                       // advance the control flag
      break;
    case 4:
      sampleI_CT4 = ADC;
      ADMUX = 0x40 + currentSensor_CT1;     // for the next-but-one conversion
      sample_index = 0;                     // reset the control flag
      break;
    default:
      sample_index = 0;                     // to prevent lockup (should never get here)
  }
}


// When using interrupt-based logic, the main processor waits in loop() until the
// dataReady flag has been set by the ADC.  Once this flag has been set, the main
// processor clears the flag and proceeds with all the processing for one pair of
// V & I samples.  It then returns to loop() to wait for the next pair to become
// available.

// If the next pair of samples become available before the processing of the
// previous pair has been completed, data could be lost.
void loop() {
  // reset the watchdog timer
  wdt_reset();

  // if in debug mode then manually check our MQTT queue
  if (ENABLE_DEBUG) { mqttMaintain(); }

  // flag is set after every pair of ADC conversions
  if (dataReady) {
    // reset the flag
    dataReady = false;

    // executed once for each pair of V&I samples
    calculateEnergy();
  }

  // see if we need to turn off the LED
  checkLed();
}

void initialiseNetwork() {
  // Determine MAC address
  byte mac[6];
  if (ENABLE_MAC_ADDRESS_ROM)
  {
    Serial.print(F("Getting MAC address from ROM: "));
    mac[0] = readRegister(MAC_I2C_ADDRESS, 0xFA);
    mac[1] = readRegister(MAC_I2C_ADDRESS, 0xFB);
    mac[2] = readRegister(MAC_I2C_ADDRESS, 0xFC);
    mac[3] = readRegister(MAC_I2C_ADDRESS, 0xFD);
    mac[4] = readRegister(MAC_I2C_ADDRESS, 0xFE);
    mac[5] = readRegister(MAC_I2C_ADDRESS, 0xFF);
  }
  else
  {
    Serial.print(F("Using static MAC address: "));
    memcpy(mac, STATIC_MAC, sizeof(mac));
  }
  char mac_address[18];
  sprintf_P(mac_address, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(mac_address);

  // Set up Ethernet
  if (ENABLE_DHCP)
  {
    Serial.print(F("Getting IP address via DHCP: "));
    Ethernet.begin(mac);
  }
  else
  {
    Serial.print(F("Using static IP address: "));
    Ethernet.begin(mac, STATIC_IP, STATIC_DNS);
  }
  Serial.println(Ethernet.localIP());

  // Generate MQTT client id, unless one is explicitly defined
  if (MQTT_CLIENT_ID == NULL)
  {
    sprintf_P(g_mqtt_client_id, PSTR("ETX-%02X%02X%02X"), mac[3], mac[4], mac[5]);  
  }
  else
  {
    memcpy(g_mqtt_client_id, MQTT_CLIENT_ID, sizeof(g_mqtt_client_id));
  }
  Serial.print(F("MQTT client id: "));
  Serial.println(g_mqtt_client_id);

  // Generate MQTT LWT topic (if required)
  if (ENABLE_MQTT_LWT)
  {
    sprintf_P(g_mqtt_lwt_topic, PSTR("%s/%s"), MQTT_LWT_BASE_TOPIC, g_mqtt_client_id);  
    Serial.print(F("MQTT LWT topic: "));
    Serial.println(g_mqtt_lwt_topic);
  }

  char topic[64];

  Serial.print(F("MQTT config topic: "));
  Serial.println(getConfigTopic(topic));

  Serial.print(F("MQTT CT1 data topic: "));
  Serial.println(getDataTopic(topic, 1));
  Serial.print(F("MQTT CT2 data topic: "));
  Serial.println(getDataTopic(topic, 2));
  Serial.print(F("MQTT CT3 data topic: "));
  Serial.println(getDataTopic(topic, 3));
  Serial.print(F("MQTT CT4 data topic: "));
  Serial.println(getDataTopic(topic, 4));
}

void initialiseCalibration() {
  // when using integer maths, calibration values that have supplied in floating point
  // form need to be rescaled.
  phaseCal_CT1 = phaseCalOffset_CT1 * 256;      // for integer maths
  phaseCal_CT2 = phaseCalOffset_CT2 * 256;      // for integer maths
  phaseCal_CT3 = phaseCalOffset_CT3 * 256;      // for integer maths
  phaseCal_CT4 = phaseCalOffset_CT4 * 256;      // for integer maths

  // define operating limits for the LP filter which identifies DC offset in the voltage
  // sample stream.  By limiting the output range, the filter always should start up
  // correctly.
  DCoffset_V = 512L * 256;                      // nominal mid-point value of ADC @ x256 scale
  DCoffset_V_min = (long)(512L - 100) * 256;    // mid-point of ADC minus a working margin
  DCoffset_V_max = (long)(512L + 100) * 256;    // mid-point of ADC plus a working margin
}

void initialiseADC() {
  // in this sketch, the ADC is free-running with a cycle time of ~104uS.
  Serial.println(F("ADC mode: FREE-RUNNING"));

  // set up the ADC to be free-running
  ADCSRA  = (1<<ADPS0)+(1<<ADPS1)+(1<<ADPS2);   // set the ADC's clock to system clock / 128
  ADCSRA |= (1 << ADEN);                        // enable the ADC

  ADCSRA |= (1<<ADATE);  // set the Auto Trigger Enable bit in the ADCSRA register.  Because
                         // bits ADTS0-2 have not been set (i.e. they are all zero), the
                         // ADC's trigger source is set to "free running mode".

  ADCSRA |=(1<<ADIE);    // set the ADC interrupt enable bit. When this bit is written
                         // to one and the I-bit in SREG is set, the
                         // ADC Conversion Complete Interrupt is activated.

  ADCSRA |= (1<<ADSC);   // start ADC manually first time
  sei();                 // Enable Global Interrupts  
}

void mqttMaintain()
{
  if (mqtt_client.loop())
  {
    // Currently connected so ensure we are ready to reconnect if it drops
    g_mqtt_backoff = 0;
    g_mqtt_last_reconnect_ms = millis();
  }
  else
  {
    // Calculate the backoff interval and check if we need to try again
    uint32_t mqtt_backoff_ms = (uint32_t)g_mqtt_backoff * MQTT_BACKOFF_SECS * 1000;
    if ((millis() - g_mqtt_last_reconnect_ms) > mqtt_backoff_ms)
    {
      Serial.print(F("Connecting to MQTT broker..."));

      if (mqttConnect()) 
      {
        Serial.println(F("success"));
      }
      else
      {
        // Reconnection failed, so backoff
        if (g_mqtt_backoff < MQTT_MAX_BACKOFF_COUNT) { g_mqtt_backoff++; }
        g_mqtt_last_reconnect_ms = millis();

        Serial.print(F("failed, retry in "));
        Serial.print(g_mqtt_backoff * MQTT_BACKOFF_SECS);
        Serial.println(F("s"));
      }
    }
  }
}

boolean mqttConnect() {
  // Attempt to connect, with a LWT if configured
  boolean success;
  if (ENABLE_MQTT_LWT)
  {
    success = mqtt_client.connect(g_mqtt_client_id, MQTT_USERNAME, MQTT_PASSWORD, g_mqtt_lwt_topic, MQTT_LWT_QOS, MQTT_LWT_RETAIN, "0");
  }
  else
  {
    success = mqtt_client.connect(g_mqtt_client_id, MQTT_USERNAME, MQTT_PASSWORD);
  }

  if (success)
  {
    // Subscribe to our config topics
    char topic[64];
    mqtt_client.subscribe(getConfigTopic(topic));
    
    // Publish LWT so anything listening knows we are alive
    if (ENABLE_MQTT_LWT)
    {
      byte lwt_payload[] = { '1' };
      mqtt_client.publish(g_mqtt_lwt_topic, lwt_payload, 1, MQTT_LWT_RETAIN);
    }
  }

  return success;
}

// MQTT call back to allow the PV router to be configured
void mqttCallback(char* topic, byte* payload, int length) {
  // tokenise the topic
  char * topicIndex;
  topicIndex = strtok(topic, "/");

  // junk the base topic if present
  if (MQTT_BASE_TOPIC != NULL)
  {
    topicIndex = strtok(NULL, "/");
  }

  // Get the topic type (convert to UPPER case)
  char * topicType = strupr(topicIndex);

  // Junk the device id (we only subscribe to our own device so no need to check)
  topicIndex = strtok(NULL, "/");

  if (ENABLE_DEBUG)
  {
    Serial.print(F("["));
    Serial.print(topicType);
    Serial.print(F("]"));
  }  

  if (strncmp(topicType, "CONF", 4) == 0)
  {
    // Parse the config type and handle
    topicIndex = strtok(NULL, "/");
    handleConfig(topicIndex, payload, length);
  }
  else
  {
    if (ENABLE_DEBUG) { Serial.println(F(" INVALID TOPIC")); }  
  }
}

void handleConfig(char * configType, byte * payload, int length) {
  // listen for control messages
  if (strncmp(configType, "scalefactor", 11) == 0) 
  {
    if (ENABLE_DEBUG) { Serial.print(F(" SCALEFACTOR:")); }

    char scaleFactor[10];
    strncpy(scaleFactor, (char*)payload, length);

    scaleFactor_CT1 = atof(scaleFactor);
    scaleFactor_CT2 = scaleFactor_CT1;
    scaleFactor_CT3 = scaleFactor_CT1;
    scaleFactor_CT4 = scaleFactor_CT1;

    if (ENABLE_DEBUG) { Serial.println(scaleFactor_CT1, 6); }
    
    // reconfigure the various scale/power factors
    updateCalculationParameters();
  } 
  else if (strncmp(configType, "divertermode", 10) == 0) 
  {
    if (ENABLE_DEBUG) { Serial.print(F(" DIVERTERMODE:")); }

    // update the output mode
    if (length == 0 || strncmp((char*)payload, "DISABLED", length) == 0) 
    {
      diverterMode = DISABLED;
      if (ENABLE_DEBUG) { Serial.println(F("DISABLED")); }
    } 
    else if (strncmp((char*)payload, "ANTIFLICKER", length) == 0) 
    {
      diverterMode = ANTI_FLICKER;
      if (ENABLE_DEBUG) { Serial.println(F("ANTI-FLICKER")); }
    } 
    else if (strncmp((char*)payload, "NORMAL", length) == 0) 
    {
      diverterMode = NORMAL;
      if (ENABLE_DEBUG) { Serial.println(F("NORMAL")); }
    } 
    else 
    {
      if (ENABLE_DEBUG) { Serial.println(F("ERROR")); }
    }
    
    // reconfigure the various scale/power factors
    updateCalculationParameters();
  } 
  else if (strncmp(configType, "diverter", 8) == 0) 
  {
    if (ENABLE_DEBUG) { Serial.print(F(" DIVERTER:")); }

    if (length == 0 || strncmp((char*)payload, "OFF", length) == 0 || strncmp((char*)payload, "0", length) == 0)
    {
      diverterEnabled = false;
      if (ENABLE_DEBUG) { Serial.println(F("OFF")); }
    }
    else if (strncmp((char*)payload, "ON", length) == 0 || strncmp((char*)payload, "1", length) == 0)
    {
      diverterEnabled = true;
      if (ENABLE_DEBUG) { Serial.println(F("ON")); }
    }
    else
    {
      if (ENABLE_DEBUG) { Serial.println(F("ERROR")); }
    }
  } 
  else 
  {
    if (ENABLE_DEBUG) { Serial.print(F(" INVALID:")); }
    if (ENABLE_DEBUG) { Serial.println(configType); }
  }
}

// This routine is called to process each pair of V & I samples.  Note that when using
// interrupt-based code, it is not necessary to delay the processing of each pair of
// samples as was done in Mk2a builds.  This is because there is no longer a strict
// alignment between the obtaining of each sample by the ADC and the processing that can
// be done by the main processor while the ADC conversion is in progress.
// When interrupts are used, the main processor and the ADC work autonomously, their
// operation being only linked via the dataReady flag.  As soon as data is made available
// by the ADC, the main processor can start to work on it immediately.
void calculateEnergy() {
  static boolean diverterNeedsToBeArmed = false;  // once per mains cycle (+ve half)
  static int samplesDuringThisCycle;              // for normalising the power in each mains cycle
  static long sumP_CT1;                           // for per-cycle summation of 'real power'
  static long sumP_CT2;
  static long sumP_CT3;
  static long sumP_CT4;
  static enum POLARITY polarityOfLastSampleV;     // for zero-crossing detection
  static long cumVdeltasThisCycle;                // for the LPF which determines DC offset (voltage)
  static long lastSampleVminusDC;                 // for the phaseCal algorithm
  static byte whenToSendCount = 0;

  // remove DC offset from the raw voltage sample by subtracting the accurate value
  // as determined by a LP filter.
  long sampleVminusDC = ((long)sampleV<<8) - DCoffset_V;

  // determine polarity, to aid the logical flow
  enum POLARITY polarityNow;
  if (sampleVminusDC > 0) {
    polarityNow = POSITIVE;
  }else {
    polarityNow = NEGATIVE;
  }

  if (polarityNow == POSITIVE)
  {
    if (beyondStartUpPhase)
    {
      if (polarityOfLastSampleV != POSITIVE)
      {
        // the diverter is armed once during each +ve half-cycle
        diverterNeedsToBeArmed = true;

        // Calculate the real power and energy during the last whole mains cycle.
        //
        // sumP contains the sum of many individual calculations of instantaneous power.  In
        // order to obtain the average power during the relevant period, sumP must first be
        // divided by the number of samples that have contributed to its value.
        //
        // The next stage would normally be to apply a calibration factor so that real power
        // can be expressed in Watts.  That's fine for floating point maths, but it's not such
        // a good idea when integer maths is being used.  To keep the numbers large, and also
        // to save time, calibration of power is omitted at this stage.  realPower is
        // therefore (1/powerCal) times larger than the actual power in Watts.
        //
        long realPower_CT1 = sumP_CT1 / samplesDuringThisCycle; // proportional to Watts
        long realPower_CT2 = sumP_CT2 / samplesDuringThisCycle; // proportional to Watts
        long realPower_CT3 = sumP_CT3 / samplesDuringThisCycle; // proportional to Watts
        long realPower_CT4 = sumP_CT4 / samplesDuringThisCycle; // proportional to Watts

        // Next, the energy content of this power rating needs to be determined.  Energy is
        // power multiplied by time, so the next step is normally to multiply by the time over
        // which the power was measured.
        //   Instanstaneous power is calculated once every mains cycle, so that's every fiftieth
        // of a second.  When integer maths is being used, this routine power-to-energy conversion
        // seems an unnecessary workload.  As all sampling periods are of similar duration (20mS),
        // it is more efficient simply to add all the power samples together, and note that their
        // sum is actually 50 times greater than it would otherwise be.
        //   Although the numerical value itself does not change, a new name is helpful so as
        // to avoid any confusion.  The 'energy' variable below is 50 * (1/powerCal) times larger
        // than the actual energy in Joules.
        //
        long realEnergy_CT1 = realPower_CT1;
        long realEnergy_CT2 = realPower_CT2;
        long realEnergy_CT3 = realPower_CT3;
        long realEnergy_CT4 = realPower_CT4;

        // Add this latest contribution to the energy bucket
        energyInBucket_CT1 += realEnergy_CT1;
        energyInBucket_CT2 += realEnergy_CT2;
        energyInBucket_CT3 += realEnergy_CT3;
        energyInBucket_CT4 += realEnergy_CT4;

        // Apply max and min limits to bucket's level.  This is to ensure correct operation
        // when conditions change, i.e. when import changes to export, and vici versa.
        // copy the value from CT1 to our grid accumulator - which we use for PV diversion
        energyInBucket += (realEnergy_CT1 * -1);
        if (energyInBucket > capacityOfEnergyBucket) {
          energyInBucket = capacityOfEnergyBucket;
        } else if (energyInBucket < 0) {
          energyInBucket = 0;
        }

        datalogCountInMainsCycles++;

        if (datalogCountInMainsCycles >= maxDatalogCountInMainsCycles)
        {
          tx_data.power_CT1 = energyInBucket_CT1 * normalisation_CT1;
          tx_data.power_CT2 = energyInBucket_CT2 * normalisation_CT2;
          tx_data.power_CT3 = energyInBucket_CT3 * normalisation_CT3;
          tx_data.power_CT4 = energyInBucket_CT4 * normalisation_CT4;

          publishReadings();

          datalogCountInMainsCycles = 0;
          energyInBucket_CT1 = 0;
          energyInBucket_CT2 = 0;
          energyInBucket_CT3 = 0;
          energyInBucket_CT4 = 0;
        }

        // clear the per-cycle accumulators for use in this new mains cycle.
        samplesDuringThisCycle = 0;
        sumP_CT1 = 0;
        sumP_CT2 = 0;
        sumP_CT3 = 0;
        sumP_CT4 = 0;

      } // end of processing that is specific to the first Vsample in each +ve half cycle

      // still processing samples where the voltage is POSITIVE ...
      if (diverterNeedsToBeArmed == true) {
        // check to see whether the diverter device can now be reliably armed
        // much easier than checking the voltage level
        if (samplesDuringThisCycle == 3) {
          // check if the diverter is enabled
          if (diverterEnabled) {
            // only update the diverter if we are outside the hysteresis range
            if (energyInBucket < lowerEnergyThreshold) {
              setDivert(DIVERT_OFF);
            } else if (energyInBucket > upperEnergyThreshold) {
              setDivert(DIVERT_ON);
            }
          } else {
            // if the diverter is not enabled then ensure it is OFF
            setDivert(DIVERT_OFF);
          }

          // clear the flag
          diverterNeedsToBeArmed = false;
        }
      }
    }
    else
    {
      // wait until the DC-blocking filters have had time to settle
      if (millis() > startUpPeriod * 1000) {
        beyondStartUpPhase = true;
        sumP_CT1 = 0;
        sumP_CT2 = 0;
        sumP_CT3 = 0;
        sumP_CT4 = 0;
        samplesDuringThisCycle = 0;
        Serial.println(F("Go!"));
      }
    }
  } // end of processing that is specific to samples where the voltage is positive

  else // the polarity of this sample is negative
  {
    if (polarityOfLastSampleV != NEGATIVE)
    {
      // This is the start of a new -ve half cycle (just after the zero-crossing point)
      //
      // This is a convenient point to update the Low Pass Filter for DC-offset removal,
      // which needs to be done right from the start (faster than * 0.01)
      long previousOffset = DCoffset_V;
      DCoffset_V = previousOffset + (cumVdeltasThisCycle>>6);
      cumVdeltasThisCycle = 0;

      // To ensure that the LPF will always start up correctly when 240V AC is available, its
      // output value needs to be prevented from drifting beyond the likely range of the
      // voltage signal.  This avoids the need to use a HPF as was done for initial Mk2 builds.
      if (DCoffset_V < DCoffset_V_min) {
        DCoffset_V = DCoffset_V_min;
      } else if (DCoffset_V > DCoffset_V_max) {
        DCoffset_V = DCoffset_V_max;
      }
    } // end of processing that is specific to the first Vsample in each -ve half cycle
  } // end of processing that is specific to samples where the voltage is positive

  // Processing for EVERY pair of samples. Most of this code is not used during the
  // start-up period, but it does no harm to leave it in place.  Accumulated values
  // are cleared when beyondStartUpPhase is set to true.
  //
  // remove most of the DC offset from the current sample (the precise value does not matter)
  long sampleIminusDC_CT1 = ((long)(sampleI_CT1-DCoffset_I))<<8;
  long sampleIminusDC_CT2 = ((long)(sampleI_CT2-DCoffset_I))<<8;
  long sampleIminusDC_CT3 = ((long)(sampleI_CT3-DCoffset_I))<<8;
  long sampleIminusDC_CT4 = ((long)(sampleI_CT4-DCoffset_I))<<8;

  // phase-shift the voltage waveform so that it aligns with the current when a
  // resistive load is used
  long  phaseShiftedSampleVminusDC_CT1 = lastSampleVminusDC + (((sampleVminusDC - lastSampleVminusDC)*phaseCal_CT1)>>8);
  long  phaseShiftedSampleVminusDC_CT2 = lastSampleVminusDC + (((sampleVminusDC - lastSampleVminusDC)*phaseCal_CT2)>>8);
  long  phaseShiftedSampleVminusDC_CT3 = lastSampleVminusDC + (((sampleVminusDC - lastSampleVminusDC)*phaseCal_CT3)>>8);
  long  phaseShiftedSampleVminusDC_CT4 = lastSampleVminusDC + (((sampleVminusDC - lastSampleVminusDC)*phaseCal_CT4)>>8);

  // calculate the "real power" in this sample pair and add to the accumulated sum
  long filtV_div4_CT1 = phaseShiftedSampleVminusDC_CT1>>2;   // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT1 = sampleIminusDC_CT1>>2;               // reduce to 16-bits (now x64, or 2^6)
  long instP_CT1 = filtV_div4_CT1 * filtI_div4_CT1;          // 32-bits (now x4096, or 2^12)
  instP_CT1 = instP_CT1>>12;                                 // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_CT1 +=instP_CT1;                                      // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)

  long filtV_div4_CT2 = phaseShiftedSampleVminusDC_CT2>>2;   // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT2 = sampleIminusDC_CT2>>2;               // reduce to 16-bits (now x64, or 2^6)
  long instP_CT2 = filtV_div4_CT2 * filtI_div4_CT2;          // 32-bits (now x4096, or 2^12)
  instP_CT2 = instP_CT2>>12;                                 // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_CT2 +=instP_CT2;                                      // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)

  long filtV_div4_CT3 = phaseShiftedSampleVminusDC_CT3>>2;   // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT3 = sampleIminusDC_CT3>>2;               // reduce to 16-bits (now x64, or 2^6)
  long instP_CT3 = filtV_div4_CT3 * filtI_div4_CT3;          // 32-bits (now x4096, or 2^12)
  instP_CT3 = instP_CT3>>12;                                 // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_CT3 +=instP_CT3;                                      // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)

  long filtV_div4_CT4 = phaseShiftedSampleVminusDC_CT4>>2;   // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT4 = sampleIminusDC_CT4>>2;               // reduce to 16-bits (now x64, or 2^6)
  long instP_CT4 = filtV_div4_CT4 * filtI_div4_CT4;          // 32-bits (now x4096, or 2^12)
  instP_CT4 = instP_CT4>>12;                                 // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_CT4 +=instP_CT4;                                      // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)

  samplesDuringThisCycle++;

  // store items for use during next loop
  cumVdeltasThisCycle += sampleVminusDC;                // for use with LP filter
  lastSampleVminusDC = sampleVminusDC;                  // required for phaseCal algorithm

  polarityOfLastSampleV = polarityNow;                            // for identification of half cycle boundaries
}
// end of calculateEnergy()

void updateCalculationParameters() {
  // calculate the power calibration factors
  float powerCal_CT1 = (235.1*(3.3/1023))*(60.606*(3.3/1023))*scaleFactor_CT1;
  float powerCal_CT2 = (235.1*(3.3/1023))*(60.606*(3.3/1023))*scaleFactor_CT2;
  float powerCal_CT3 = (235.1*(3.3/1023))*(60.606*(3.3/1023))*scaleFactor_CT3;
  float powerCal_CT4 = (235.1*(3.3/1023))*(60.606*(3.3/1023))*scaleFactor_CT4;

  // calculate the normalisation factor to actually apply to our readings
  normalisation_CT1 = powerCal_CT1 / maxDatalogCountInMainsCycles;
  normalisation_CT2 = powerCal_CT2 / maxDatalogCountInMainsCycles;
  normalisation_CT3 = powerCal_CT3 / maxDatalogCountInMainsCycles;
  normalisation_CT4 = powerCal_CT4 / maxDatalogCountInMainsCycles;

  // for the flow of energy at the 'grid' connection point (CT1)
  capacityOfEnergyBucket = (long)SWEETZONE_IN_JOULES * CYCLES_PER_SECOND * (1/powerCal_CT1);

  if (ENABLE_DEBUG) {
    Serial.println(F("** Calibration parameters **"));
    Serial.print(F("    scaleFactor_CT1             = ")); Serial.println(scaleFactor_CT1, 6);
    Serial.print(F("    scaleFactor_CT2             = ")); Serial.println(scaleFactor_CT2, 6);
    Serial.print(F("    scaleFactor_CT3             = ")); Serial.println(scaleFactor_CT3, 6);
    Serial.print(F("    scaleFactor_CT4             = ")); Serial.println(scaleFactor_CT4, 6);
    Serial.print(F("    offsetOfEnergyThresholds    = ")); Serial.println(offsetOfEnergyThresholdsInAFmode);
    Serial.print(F("    capacityOfEnergyBucket      = ")); Serial.println(capacityOfEnergyBucket);
  }
    
  // calculate the energy bucket thresholds for use by the diverter
  if (ENABLE_DEBUG) { Serial.print(F("    diverter mode               = ")); }
  if (diverterMode == ANTI_FLICKER)
  {
    if (ENABLE_DEBUG) { Serial.println(F("ANTI-FLICKER")); }
    
    // settings for anti-flicker mode
    lowerEnergyThreshold = capacityOfEnergyBucket * (0.5 - offsetOfEnergyThresholdsInAFmode);
    upperEnergyThreshold = capacityOfEnergyBucket * (0.5 + offsetOfEnergyThresholdsInAFmode);
  }
  else if (diverterMode == NORMAL)
  {
    if (ENABLE_DEBUG) { Serial.println(F("NORMAL")); }

    // settings for normal mode
    lowerEnergyThreshold = capacityOfEnergyBucket * 0.5;
    upperEnergyThreshold = capacityOfEnergyBucket * 0.5;
  }
  else
  {
    if (ENABLE_DEBUG) { Serial.println(F("DISABLED")); }

    // settings for disabled
    lowerEnergyThreshold = 0;
    upperEnergyThreshold = 0;
  }
  
  if (ENABLE_DEBUG) {
    Serial.print(F("    lowerEnergyThreshold        = "));
    Serial.println(lowerEnergyThreshold);
    Serial.print(F("    upperEnergyThreshold        = "));
    Serial.println(upperEnergyThreshold);
  }
}

void publishReadings() {
  // check our DHCP lease is still ok
  Ethernet.maintain();

  // Check our MQTT broker connection is still ok
  mqttMaintain();

  // publish our readings to MQTT (if our broker connection is up)
  if (mqtt_client.connected()) {
    char topic[32];
    publishReading(getDataTopic(topic, 1), tx_data.power_CT1);
    publishReading(getDataTopic(topic, 2), tx_data.power_CT2);
    publishReading(getDataTopic(topic, 3), tx_data.power_CT3);
    publishReading(getDataTopic(topic, 4), tx_data.power_CT4);

    digitalWrite(LED_pin, LED_ON);
    LED_onAt = millis();
  }

  // dump readings to serial for debugging
  if (ENABLE_DEBUG) {
    Serial.print(F("CT1 = "));
    Serial.print(tx_data.power_CT1);
    Serial.print(F(", CT2 = "));
    Serial.print(tx_data.power_CT2);
    Serial.print(F(", CT3 = "));
    Serial.print(tx_data.power_CT3);
    Serial.print(F(", CT4 = "));
    Serial.print(tx_data.power_CT4);
    Serial.println();
  }
}

void checkLed() {
  if ((millis() - LED_onAt) > LED_onMs) {
    digitalWrite(LED_pin, LED_OFF);
    LED_onAt = 0;
  }
}

void setDivert(int divert) {
  // only publish an update if the divert state has changed
  if (divert != divertCurrent) {
    char topic[32];
    publishDivert(getDivertTopic(topic), divert);
    divertCurrent = divert;
  }
}

char * getConfigTopic(char topic[])
{
  if (MQTT_BASE_TOPIC == NULL)
  {
    sprintf_P(topic, PSTR("conf/%s/+"), g_mqtt_client_id);
  }
  else
  {
    sprintf_P(topic, PSTR("%s/conf/%s/+"), MQTT_BASE_TOPIC, g_mqtt_client_id);
  }
  return topic;
}

char * getDataTopic(char topic[], uint8_t ct)
{
  if (MQTT_BASE_TOPIC == NULL)
  {
    sprintf_P(topic, PSTR("stat/%s/%d"), g_mqtt_client_id, ct);
  }
  else
  {
    sprintf_P(topic, PSTR("%s/stat/%s/%d"), MQTT_BASE_TOPIC, g_mqtt_client_id, ct);
  }
  return topic;
}

char * getDivertTopic(char topic[])
{
  if (MQTT_BASE_TOPIC == NULL)
  {
    sprintf_P(topic, PSTR("stat/%s/divert"), g_mqtt_client_id);
  }
  else
  {
    sprintf_P(topic, PSTR("%s/stat/%s/divert"), MQTT_BASE_TOPIC, g_mqtt_client_id);
  }
  return topic;
}

void publishReading(char * topic, int reading) {
  // build the MQTT payload
  char payload[16];
  sprintf_P(payload, PSTR("%i"), reading);

  // publish to the MQTT broker
  mqtt_client.publish(topic, payload);
}

void publishDivert(char * topic, int divert) {
  // build the MQTT payload
  char payload[2];
  sprintf_P(payload, PSTR("%i"), divert);

  // publish to the MQTT broker
  mqtt_client.publish(topic, payload);
}

// Read 1 byte from I2C device register
uint8_t readRegister(int adr, uint8_t reg) {
  Wire.beginTransmission(adr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(adr, 1);
  while (!Wire.available()) {}
  return Wire.read();
}

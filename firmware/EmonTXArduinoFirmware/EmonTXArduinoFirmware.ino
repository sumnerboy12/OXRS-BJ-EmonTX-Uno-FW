// Code originally by Robin Emley (calypso_rae on Open Energy Monitor Forum) - September 2013
// Updated November 2013 to include analog and LED pins for the emonTx V3 by Glyn Hudson
//
// Updated July 2014 to send readings via MQTT by Ben Jones
// Updated October 2014 to include PV router functionality by Ben Jones
// Updated January 2015 to add support for remote config of scale factor (by MQTT)
// Updated November 2021 to add OXRS_MQTT and streamline code
//
// The interrupt-based kernel for this sketch was kindly provided by Jorg Becker.

/*--------------------------- Firmware -----------------------------------*/
#define FW_NAME       "EmonTXArduinoFirmware"
#define FW_MAKER      "Ben Jones"
#define FW_VERSION    "1.0.0"

/*--------------------------- Configuration ------------------------------*/
// Should be no user configuration in this file, everything should be in;
#include "config.h"
#include "calibration.h"

/*-------------------------- Libraries --------------------------*/
#include <Ethernet.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <OXRS_MQTT.h>

/*-------------------------- Constants --------------------------*/
#define     SERIAL_BAUD_RATE          115200
#define     DHCP_TIMEOUT_MS           15000
#define     DHCP_RESPONSE_TIMEOUT_MS  4000

// hardware pins (sensors are analogue inputs)
const byte  CURRENT_SENSOR[CT_COUNT]  = { 1, 2, 3, 4 };
const byte  VOLTAGE_SENSOR            = 0;
const byte  LED_PIN                   = 9;

/*-------------------------- Global Variables -------------------*/
// Calibration factor
float scaleFactor             = DEFAULT_SCALE_FACTOR;

// Diverter can be turned on/off
boolean diverterEnabled       = DEFAULT_DIVERTER_ENABLED;
boolean diverterState         = false;

/*-------------------------- Internal Datatypes -----------------*/
enum POLARITY                 { NEGATIVE, POSITIVE };

typedef struct 
{
  int sampleI;
  long lpf;
  long sumP;
  long sumE;
} EMONTX_DATA;
EMONTX_DATA g_data[CT_COUNT];

/*-------------------------- Energy Calculation -----------------*/
// Some of these variables are used in multiple blocks so cannot be static.
// For integer maths, many variables need to be 'long'
const byte startUpPeriodMs    = 3000;   // to allow LP filter to settle
boolean beyondStartUpPeriod   = false;  // start-up delay, allows things to settle

// Low pass filter
const int DCoffset_I          = 512;    // nominal mid-point value of ADC @ x1 scale
long DCoffset_V;
long DCoffset_V_min;
long DCoffset_V_max;

// Power calibration factor (see calibration.h)
float powerCal;

// Voltage sample
int sampleV;

// Diverter thresholds and counter
long capacityOfEnergyBucket;            // depends on powerCal, frequency & the 'sweetzone' size
long energyThreshold;                   // for turning diverter on/off

// Enhanced polarity (zero-crossing) detection
#define CONSECUTIVE_SAMPLES_FOR_CONFIRMED_POLARITY 1
enum POLARITY confirmedPolarity;
enum POLARITY confirmedPolarityPrevious;

/*-------------------------- Data logger ------------------------*/
// For interaction between the main processor and the ISR
volatile boolean dataReady    = false;

// Data logger
const int maxDatalogCountInMainsCycles = DATALOG_PERIOD_SECS * CYCLES_PER_SEC;
int datalogCountInMainsCycles = 0;
float datalogNormalisationFactor;

// Pulse LED when publishing data
unsigned long LED_onAt;

/*-------------------------- Global Objects ---------------------*/
EthernetClient _client;

PubSubClient _mqttClient(_client);
OXRS_MQTT _mqtt(_mqttClient);

/*-------------------------- MQTT callbacks ---------------------*/
void _mqttCallback(char * topic, byte * payload, int length)
{
  _mqtt.receive(topic, payload, length);
}

void _mqttConnected()
{
  Serial.println(F("[emon] mqtt connected"));
}

void _mqttDisconnected(int state)
{
  Serial.print(F("[emon] mqtt disconnected (state "));
  Serial.print(state);
  Serial.println(F(")"));
}

void _mqttConfig(JsonVariant json)
{
  boolean updateParams = false;

  if (json.containsKey("scaleFactor"))
  {
    scaleFactor = json["scaleFactor"].as<float>();    
    updateParams = true;
  }

  if (json.containsKey("diverterEnabled"))
  {
    diverterEnabled = json["diverterEnabled"].as<boolean>();
    updateParams = true;
  }

  if (updateParams){ updateCalculationParameters(); }
}

/*-------------------------- Program ----------------------------*/
void setup()
{
  // Start the serial interface and display the firmware details
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println();
  Serial.println(F("========================================"));
  Serial.print  (F("FIRMWARE: ")); Serial.println(FW_NAME);
  Serial.print  (F("MAKER:    ")); Serial.println(FW_MAKER);
  Serial.print  (F("VERSION:  ")); Serial.println(FW_VERSION);
  Serial.println(F("========================================"));

  // setup indicator LED
  pinMode(LED_PIN, OUTPUT);

  // initialise ethernet
  byte mac[6];
  initialiseEthernet(mac);

  // initialise MQTT
  initialiseMqtt(mac);

  // setup our calibration variables
  initialiseCalibration();

  // initialse the ADC and setup the interrupt handler
  initialiseADC();

  // update all our config parameters, using the default/startup values
  updateCalculationParameters();
}

// An Interrupt Service Routine is defined in which the ADC is instructed to perform
// a conversion of the voltage signal and each of the signals for current.  A "data ready"
// flag is set after each voltage conversion has been completed, it being the last one in
// the sequence.

// Samples for current are taken first because the phase of the waveform for current is
// generally slightly advanced relative to the waveform for voltage.  The data ready flag
// is cleared within loop().

// This Interrupt Service Routine is for use when the ADC is in the free-running mode.
// It is executed whenever an ADC conversion has finished, approx every 104 us.  In
// free-running mode, the ADC has already started its next conversion by the time that
// the ISR is executed.  The ISR therefore needs to "look ahead".

// At the end of conversion Type N, conversion Type N+1 will start automatically.  The ISR
// which runs at this point therefore needs to capture the results of conversion Type N ,
// and set up the conditions for conversion Type N+2, and so on.
//
ISR(ADC_vect)
{
  static unsigned char sample_index = 0;

  switch (sample_index)
  {
    case 0:
      sampleV = ADC;
      ADMUX = 0x40 + CURRENT_SENSOR[1];     // set up the next-but-one conversion
      sample_index++;                       // advance the control flag
      dataReady = true;
      break;
    case 1:
      g_data[0].sampleI = ADC;
      ADMUX = 0x40 + CURRENT_SENSOR[2];     // for the next-but-one conversion
      sample_index++;                       // advance the control flag
      break;
    case 2:
      g_data[1].sampleI = ADC;
      ADMUX = 0x40 + CURRENT_SENSOR[3];     // for the next-but-one conversion
      sample_index++;                       // advance the control flag
      break;
    case 3:
      g_data[2].sampleI = ADC;
      ADMUX = 0x40 + VOLTAGE_SENSOR;        // for the next-but-one conversion
      sample_index++;                       // advance the control flag
      break;
    case 4:
      g_data[3].sampleI = ADC;
      ADMUX = 0x40 + CURRENT_SENSOR[0];     // for the next-but-one conversion
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
void loop()
{
  // check our DHCP lease is still ok
  Ethernet.maintain();

  // check our MQTT broker connection is still ok
  _mqtt.loop();

  // flag is set after every pair of ADC conversions
  if (dataReady)
  {
    // reset the flag
    dataReady = false;

    // executed once for each pair of V&I samples
    calculateEnergy();
  }

  // see if we need to turn off the LED
  checkLed();
}

void initialiseEthernet(byte * mac) 
{
  // Use static MAC address (since MAC address ROM is disabled)
  memcpy(mac, STATIC_MAC, sizeof(STATIC_MAC));

  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.print(F("[emon] mac address: "));
  Serial.println(mac_display);

  // Set up Ethernet
  Serial.print(F("[emon] ip address:  "));
  if (Ethernet.begin(mac, DHCP_TIMEOUT_MS, DHCP_RESPONSE_TIMEOUT_MS))
  {
    Serial.println(Ethernet.localIP());
  }
  else
  {
    Serial.println(F("none"));
  }
}

void initialiseMqtt(byte * mac)
{
  // Set the broker connection details (hard coded)
  _mqtt.setBroker(MQTT_BROKER, MQTT_PORT);
  _mqtt.setAuth(MQTT_USERNAME, MQTT_PASSWORD);
  
  // Set the client ID to last 3 bytes of the MAC address
  char clientId[32];
  sprintf_P(clientId, PSTR("%02x%02x%02x"), mac[3], mac[4], mac[5]);
  _mqtt.setClientId(clientId);
  
  // Register our callbacks
  _mqtt.onConnected(_mqttConnected);
  _mqtt.onDisconnected(_mqttDisconnected);
  _mqtt.onConfig(_mqttConfig);

  // Shrink the internal MQTT buffer size since our heap is close to capacity
  // and we only expect to receive short config payloads
  _mqttClient.setBufferSize(128);
  
  // Start listening for MQTT messages
  _mqttClient.setCallback(_mqttCallback);
}

void initialiseCalibration()
{
  // calculate the power calibration factor
  powerCal = (VOLTAGE_CALIBRATION * (ADC_REF_VOLTAGE / ADC_STEPS)) * (CURRENT_CALIBRATION * (ADC_REF_VOLTAGE / ADC_STEPS));

  // define operating limits for the LP filter which identifies DC offset in the voltage
  // sample stream.  By limiting the output range, the filter always should start up
  // correctly.
  DCoffset_V = 512L * 256;                      // nominal mid-point value of ADC @ x256 scale
  DCoffset_V_min = (long)(512L - 100) * 256;    // mid-point of ADC minus a working margin
  DCoffset_V_max = (long)(512L + 100) * 256;    // mid-point of ADC plus a working margin
}

void initialiseADC()
{
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

// This routine is called to process each pair of V & I samples.  Note that when using
// interrupt-based code, it is not necessary to delay the processing of each pair of
// samples as was done in Mk2a builds.  This is because there is no longer a strict
// alignment between the obtaining of each sample by the ADC and the processing that can
// be done by the main processor while the ADC conversion is in progress.
// When interrupts are used, the main processor and the ADC work autonomously, their
// operation being only linked via the dataReady flag.  As soon as data is made available
// by the ADC, the main processor can start to work on it immediately.
void calculateEnergy()
{
  // for normalising the power in each mains cycle
  static int samplesDuringThisCycle;
  // for arming the diverter load
  static int samplesDuringNegativeHalfOfCycle;

  // for diverter
  static long energyInBucket;
  static long energyInBucket_prediction;

  // for the LPF which determines DC offset (voltage)
  static long cumVdeltasThisCycle;

  // remove DC offset from the raw voltage sample by subtracting the accurate value
  // as determined by a LP filter.
  long sampleVminusDC = ((long)sampleV<<8) - DCoffset_V;

  // determine polarity, to aid the logical flow
  checkPolarity(sampleVminusDC > 0 ? POSITIVE : NEGATIVE);

  if (confirmedPolarity == POSITIVE)
  {
    if (confirmedPolarityPrevious != POSITIVE)
    {
      if (beyondStartUpPeriod)
      {
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
        for (int i = 0; i < CT_COUNT; i++)
        {
          // proportional to Watts
          long realPower = g_data[i].sumP / samplesDuringThisCycle;

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
          long realEnergy = realPower;

          // Add this latest contribution to the energy accumulators (for data logging)
          g_data[i].sumE += realEnergy;

          // Copy the (inverted) grid value to our diverter accumulator
          if (i == CT_GRID)
          {
            energyInBucket += (realEnergy * -1);
          }
        }
        
        // Apply max and min limits to bucket's level.  This is to ensure correct operation
        // when conditions change, i.e. when import changes to export, and vici versa.
        if (energyInBucket > capacityOfEnergyBucket)
        {
          energyInBucket = capacityOfEnergyBucket;
        }
        else if (energyInBucket < 0)
        {
          energyInBucket = 0;
        }

        // Do we need to publish our telemetry data?
        if (++datalogCountInMainsCycles >= maxDatalogCountInMainsCycles)
        {
          publishTelemetry();

          for (int i = 0; i < CT_COUNT; i++) { g_data[i].sumE = 0; }
          datalogCountInMainsCycles = 0;
        }

        // clear the per-cycle accumulators for use in this new mains cycle
        for (int i = 0; i < CT_COUNT; i++) { g_data[i].sumP = 0; }
        
        samplesDuringThisCycle = 0;
        samplesDuringNegativeHalfOfCycle = 0;
      }
      else
      {
        // wait until the DC-blocking filters have had time to settle
        if (millis() > startUpPeriodMs)
        {
          for (int i = 0; i < CT_COUNT; i++) { g_data[i].sumP = 0; }
          
          samplesDuringThisCycle = 0;
          samplesDuringNegativeHalfOfCycle = 0;

          energyInBucket = 0;
          energyInBucket_prediction = 0;
          
          beyondStartUpPeriod = true;
          Serial.println(F("[emon] running!"));
        }
      }
    } // end of processing that is specific to the first Vsample in each +ve half cycle 

    // still processing samples where the voltage is POSITIVE ...    
    // (in this go-faster code, the action from here has moved to the negative half of the cycle)
    
  } // end of processing that is specific to samples where the voltage is positive

  else // the polarity of this sample is negative
  {
    if (confirmedPolarityPrevious != NEGATIVE)
    {
      // This is the start of a new -ve half cycle (just after the zero-crossing point)
      //
      // This is a convenient point to update the Low Pass Filter for DC-offset removal,
      // which needs to be done right from the start (faster than * 0.01)
      //
      long previousOffset = DCoffset_V;
      DCoffset_V = previousOffset + (cumVdeltasThisCycle>>6);
      cumVdeltasThisCycle = 0;

      // To ensure that the LPF will always start up correctly when 240V AC is available, its
      // output value needs to be prevented from drifting beyond the likely range of the
      // voltage signal.  This avoids the need to use a HPF as was done for initial Mk2 builds.
      //
      if (DCoffset_V < DCoffset_V_min)
      {
        DCoffset_V = DCoffset_V_min;
      } 
      else if (DCoffset_V > DCoffset_V_max)
      {
        DCoffset_V = DCoffset_V_max;
      }

      // The average power that has been measured during the first half of this mains cycle can now be used
      // to predict the energy state at the end of this mains cycle.  That prediction will be used to alter 
      // the state of the load as necessary. The arming signal for the triac can't be set yet - that must 
      // wait until the voltage has advanced further beyond the -ve going zero-crossing point.
      //
      long gridAveragePower = g_data[CT_GRID].sumP / samplesDuringThisCycle;

      // To avoid repetitive and unnecessary calculations, the increase in energy during each mains cycle is
      // deemed to be numerically equal to the average power.  The predicted value for the energy state at the 
      // end of this mains cycle will therefore be the known energy state at its start plus the average power 
      // as measured. Although the average power has been determined over only half a mains cycle, the correct
      // number of contributing sample sets has been used so the result can be expected to be a true measurement 
      // of average power, not half of it.  
      //
      energyInBucket_prediction = energyInBucket + gridAveragePower;
      
    } // end of processing that is specific to the first Vsample in each -ve half cycle

    // check to see whether the diverter can now be reliably armed
    if (beyondStartUpPeriod && samplesDuringNegativeHalfOfCycle == 3)
    {
      if (diverterEnabled)
      {
        // turn the diverter on/off depending on how full the energy bucket is
        updateDiverterState(energyInBucket_prediction >= energyThreshold);
      }
      else
      {
        // ensure the diverter is off if it has been disabled
        updateDiverterState(false);
      }
    }

    samplesDuringNegativeHalfOfCycle++;
  } // end of processing that is specific to samples where the voltage is negative

  // Processing for EVERY pair of samples. Most of this code is not used during the
  // start-up period, but it does no harm to leave it in place.  Accumulated values
  // are cleared when beyondStartUpPeriod is set to true.
  //
  // calculate the "real power" in this sample pair and add to the accumulated sum
  for (int i = 0; i < CT_COUNT; i++)
  {
    // remove most of the DC offset from the current sample (the precise value does not matter)
    long sampleIminusDC = ((long)(g_data[i].sampleI - DCoffset_I))<<8;

    // extra filtering to offset the HPF effect of the CT
    g_data[i].lpf = (g_data[i].lpf + LPF_ALPHA * (sampleIminusDC - g_data[i].lpf));
    sampleIminusDC += (LPF_GAIN * g_data[i].lpf);
    
    // calculate the "real power" in this sample pair and add to the accumulated sum
    long filtV = sampleVminusDC>>2;   // reduce to 16-bits (now x64, or 2^6)
    long filtI = sampleIminusDC>>2;   // reduce to 16-bits (now x64, or 2^6)

    long instP = filtV * filtI;       // 32-bits (now x4096, or 2^12)
    instP = instP>>12;                // scaling is now x1, as for Mk2 (V_ADC x I_ADC)       
    g_data[i].sumP += instP;          // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  }

  // increment sample count
  samplesDuringThisCycle++;

  // for use with LP filter
  cumVdeltasThisCycle += sampleVminusDC;

  // for identification of half cycle boundaries
  confirmedPolarityPrevious = confirmedPolarity;
} // end of calculateEnergy()

void updateCalculationParameters()
{  
  // calculate the datalogNormalisationFactor to apply to our readings before publishing
  datalogNormalisationFactor = (powerCal * scaleFactor) / maxDatalogCountInMainsCycles;
    
  // calculate the energy bucket size for the flow of energy at the 'grid' connection point
  capacityOfEnergyBucket = (long)SWEETZONE_IN_JOULES * CYCLES_PER_SEC * (1/(powerCal * scaleFactor));

  // calculate the energy bucket thresholds for use by the diverter
  energyThreshold = capacityOfEnergyBucket * 0.5;
}

// This routine prevents a zero-crossing point from being declared until 
// a certain number of consecutive samples in the 'other' half of the 
// waveform have been encountered.  
void checkPolarity(POLARITY polarity)
{
  static byte sampleCount = 0;
  
  if (polarity != confirmedPolarityPrevious)
  {
    sampleCount++;
  } 
  else
  {
    sampleCount = 0;
  }
    
  if (sampleCount > CONSECUTIVE_SAMPLES_FOR_CONFIRMED_POLARITY)
  {
    sampleCount = 0;
    confirmedPolarity = polarity;
  }
}

void publishTelemetry()
{
  // publish our readings to MQTT
  StaticJsonDocument<128> json;
  
  char key[6];
  for (int i = 0; i < CT_COUNT; i++)
  {
    sprintf_P(key, PSTR("ct%d"), i + 1);
    json[key] = g_data[i].sumE * datalogNormalisationFactor;
  }

  if (_mqtt.publishTelemetry(json.as<JsonVariant>()))
  {
    digitalWrite(LED_PIN, HIGH);
    LED_onAt = millis();
  }
}

void updateDiverterState(boolean state)
{
  // only publish an update if the diverter state has changed
  if (state != diverterState)
  {
    StaticJsonDocument<32> json;
    json["divert"] = state;
    
    if (_mqtt.publishStatus(json.as<JsonVariant>()))
    {
      diverterState = state;
    }
  }
}

void checkLed()
{
  if ((millis() - LED_onAt) > LED_PULSE_MS)
  {
    digitalWrite(LED_PIN, LOW);
    LED_onAt = 0;
  }
}

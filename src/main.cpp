// Signal K application template file.
//
// Based on Halmet HAT Labs code - This will now read a BMP280, 2 digital 1-wire sensors, Tacho on D1, Bilge monitor on D2, Fuel/Tank monitor on A1, Voltage on A2, Engine temp with Ohm to K on A3 and Oil Pressure Ohm to Pa on A4.
// Code in test as of Sept 2025
// Boating With The Baileys

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NMEA2000_esp32.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>

#include "n2k_senders.h"
#include "sensesp/net/discovery.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/system_status_led.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder

#include "sensesp_onewire/onewire_temperature.h"
#define ENABLE_SIGNALK


#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"

using namespace sensesp;
using namespace halmet;
using namespace sensesp::onewire;


class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate RPM to m3/s
    clear_samples();
    // addSample(CurveInterpolator::Sample(RPM, m3/s));
    add_sample(CurveInterpolator::Sample(500, 0.00000011));
    add_sample(CurveInterpolator::Sample(1000, 0.00000019));
    add_sample(CurveInterpolator::Sample(1500, 0.0000003));
    add_sample(CurveInterpolator::Sample(1800, 0.00000041));
    add_sample(CurveInterpolator::Sample(2000, 0.00000052));
    add_sample(CurveInterpolator::Sample(2200, 0.00000066));
    add_sample(CurveInterpolator::Sample(2400, 0.00000079));
    add_sample(CurveInterpolator::Sample(2600, 0.00000097));
    add_sample(CurveInterpolator::Sample(2800, 0.00000124));
    add_sample(CurveInterpolator::Sample(3000, 0.00000153));
    add_sample(CurveInterpolator::Sample(3200, 0.00000183));
    add_sample(CurveInterpolator::Sample(3400, 0.000002));
    add_sample(CurveInterpolator::Sample(3800, 0.00000205));  
  }
};


/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.


Adafruit_BMP280 bmp280;

float read_temp_callback() { return (bmp280.readTemperature() + 273.15);}
float read_pressure_callback() { return (bmp280.readPressure());}

tNMEA2000* nmea2000;
elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

TwoWire* i2c;
Adafruit_SSD1306* display;

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

// Set the ADS1115 GAIN to adjust the analog input voltage range.
// On HALMET, this refers to the voltage range of the ADS1115 input
// AFTER the 33.3/3.3 voltage divider.

// GAIN_TWOTHIRDS: 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// GAIN_ONE:       1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// GAIN_TWO:       2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// GAIN_FOUR:      4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// GAIN_EIGHT:     8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// GAIN_SIXTEEN:   16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

const adsGain_t kADS1115Gain = GAIN_ONE;

/////////////////////////////////////////////////////////////////////
// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.
const int kTestOutputFrequency = 380;
#endif


/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // These calls can be used for fine-grained control over the logging level.
  // esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);

  Serial.begin(115200);

  /////////////////////////////////////////////////////////////////////
  // Initialize the application framework

  // Construct the global SensESPApp() object
  BUILDER_CLASS builder;
  sensesp_app = (&builder)
                    // EDIT: Set a custom hostname for the app.
                    ->set_hostname("halmet")
                    // EDIT: Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    ->set_sk_server("192.168.0.213:3000", 3000)
                    // EDIT: Enable OTA updates with a password.
                    //->enable_ota("my_ota_password")
                    ->enable_uptime_sensor()
                    ->enable_ip_address_sensor()
                    ->enable_wifi_signal_sensor()
                    ->get_app();

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();

  ads1115->setGain(kADS1115Gain);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  // Set the LEDC peripheral to a 13-bit resolution
  ledcAttach(kTestOutputPin, kTestOutputFrequency, 13);
  // Set the duty cycle to 50%
  // Duty cycle value is calculated based on the resolution
  // For 13-bit resolution, max value is 8191, so 50% is 4096
  ledcWrite(kTestOutputPin, 4096);
#endif

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20231229",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "HALMET",    // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    71  // Default N2k node address
  );
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  // Initialize the OLED display
  bool display_present = InitializeSSD1306(sensesp_app->get(), &display, i2c);

  ///  1-Wire Temp Sensors ///
  /// Exhaust Temp Sensors ///

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(4);

  auto* exhaust_temp =
      new OneWireTemperature(dts, 1000, "/Exhaust Temperature/oneWire");

    ConfigItem(exhaust_temp)
      ->set_title("Exhaust Temperature Sender")
      ->set_description("Device ID of the engine exhaust sender")
      ->set_sort_order(100);

    auto exhaust_temp_calibration =
      new Linear(1.0, 0.0, "/Exhaust_Temperature/linear");

    ConfigItem(exhaust_temp_calibration)
      ->set_title("Exhaust Temperature Calibration")
      ->set_description("Calibration for the exhaust temperature sensor")
      ->set_sort_order(200);

    auto exhaust_temp_sk_output = new SKOutputFloat(
      "propulsion.engine.1.exhaustTemperature", "/Exhaust_Temperature/skPath");
     
     ConfigItem(exhaust_temp_sk_output)
      ->set_title("Exhaust Temperature Signal K Path")
      ->set_description("Signal K path for the exhaust temperature")
      ->set_sort_order(300);

    exhaust_temp->connect_to(exhaust_temp_calibration)
      ->connect_to(exhaust_temp_sk_output);

/// Oil Temp Sensors ///

  auto oil_temp =
      new OneWireTemperature(dts, 1000, "/Oil Temperature/oneWire");

    ConfigItem(oil_temp)
      ->set_title("Oil Temperature Sender")
      ->set_description("Device ID of the engine oil sender")
      ->set_sort_order(100);

    auto oil_temp_calibration =
      new Linear(1.0, 0.0, "/oil_Temperature/linear");

    ConfigItem(oil_temp_calibration)
      ->set_title("Oil Temperature Calibration")
      ->set_description("Calibration for the oil temperature sensor")
      ->set_sort_order(200);

    auto oil_temp_sk_output = new SKOutputFloat(
      "propulsion.engine.1.oilTemperature", "/oil_Temperature/skPath");
     
     ConfigItem(oil_temp_sk_output)
      ->set_title("Oil Temperature Signal K Path")
      ->set_description("Signal K path for the oil temperature")
      ->set_sort_order(300);

    oil_temp->connect_to(oil_temp_calibration)
      ->connect_to(oil_temp_sk_output);
  
  
  ///////////////////////////////////////////////////////////////////
  // Analog inputs

  bool enable_signalk_output = true;

  // Connect the tank senders.
  // EDIT: To enable more tanks, uncomment the lines below.
  auto tank_a1_volume = ConnectTankSender(ads1115, 0, "Fuel", "fuel.main", 3000,
                                          enable_signalk_output);
  // auto tank_a3_volume = ConnectTankSender(ads1115, 1, "A2");
  auto engine_OilPressure = ConnectEngineOilSender(ads1115, 2, "Engine Oil Pressure", "oilPressure", 1000,
                                          enable_signalk_output);
  auto engine_temperature = ConnectEngineSender(ads1115, 3, "Engine Temperature", "temperature", 1000,
                                          enable_signalk_output);

#ifdef ENABLE_NMEA2000_OUTPUT
  // Tank 1, instance 0. Capacity 150 liters. You can change the capacity
  // in the web UI as well.
  // EDIT: Make sure this matches your tank configuration above.
  N2kFluidLevelSender* tank_a1_sender = new N2kFluidLevelSender(
      "/Tanks/Fuel/NMEA 2000", 0, N2kft_Fuel, 150, nmea2000);

  ConfigItem(tank_a1_sender)
      ->set_title("Tank A1 NMEA 2000")
      ->set_description("NMEA 2000 tank sender for tank A1")
      ->set_sort_order(3005);

  tank_a1_volume->connect_to(&(tank_a1_sender->tank_level_));
#endif  // ENABLE_NMEA2000_OUTPUT

  if (display_present) {
    // EDIT: Duplicate the lines below to make the display show all your tanks.
    tank_a1_volume->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 2, "Tank A1", 100 * value); }));
  }

  // Read the voltage level of analog input A2
  auto a2_voltage = new ADS1115VoltageInput(ads1115, 1, "/Voltage A2");

  ConfigItem(a2_voltage)
      ->set_title("Analog Voltage A2")
      ->set_description("Voltage level of analog input A2")
      ->set_sort_order(3000);

  a2_voltage->connect_to(new LambdaConsumer<float>(
      [](float value) { debugD("Voltage A2: %f", value); }));

  // If you want to output something else than the voltage value,
  // you can insert a suitable transform here.
  // For example, to convert the voltage to a distance with a conversion
  // factor of 0.17 m/V, you could use the following code:
  // auto a2_distance = new Linear(0.17, 0.0);
  // a2_voltage->connect_to(a2_distance);

  a2_voltage->connect_to(
      new SKOutputFloat("sensors.a2.voltage", "Analog Voltage A2",
                        new SKMetadata("V", "Analog Voltage A2")));
  // Example of how to output the distance value to Signal K.
  // a2_distance->connect_to(
  //     new SKOutputFloat("sensors.a2.distance", "Analog Distance A2",
  //                       new SKMetadata("m", "Analog Distance A2")));

  ///////////////////////////////////////////////////////////////////
  // Digital alarm inputs

  // EDIT: More alarm inputs can be defined by duplicating the lines below.
  // Make sure to not define a pin for both a tacho and an alarm.
  auto alarm_d2_input = ConnectAlarmSender(kDigitalInputPin2, "engineBilge");
  auto alarm_d3_input = ConnectAlarmSender(kDigitalInputPin3, "D3");
  // auto alarm_d4_input = ConnectAlarmSender(kDigitalInputPin4, "D4");

  // Update the alarm states based on the input value changes.
  // EDIT: If you added more alarm inputs, uncomment the respective lines below.
  alarm_d2_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  // In this example, alarm_d3_input is active low, so invert the value.
  auto alarm_d3_inverted = alarm_d3_input->connect_to(
      new LambdaTransform<bool, bool>([](bool value) { return !value; }));
  alarm_d3_inverted->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  // alarm_d4_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));

  // EDIT: This example connects the D2 alarm input to the low oil pressure
  // warning. Modify according to your needs.
  N2kEngineParameterDynamicSender* engine_dynamic_sender =
      new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0,
                                          nmea2000);

  ConfigItem(engine_dynamic_sender)
      ->set_title("Engine 1 Dynamic")
      ->set_description("NMEA 2000 dynamic engine parameters for engine 1")
      ->set_sort_order(3010);

  alarm_d2_input->connect_to(engine_dynamic_sender->low_oil_pressure_);

  // This is just an example -- normally temperature alarms would not be
  // active-low (inverted).
  alarm_d3_inverted->connect_to(engine_dynamic_sender->over_temperature_);

  // FIXME: Transmit the alarms over SK as well.

  ///////////////////////////////////////////////////////////////////
    // Digital tacho inputs

  // Connect the tacho senders. Engine name is "main".
  // EDIT: More tacho inputs can be defined by duplicating the line below.
  auto tacho_d1_frequency = ConnectTachoSender(kDigitalInputPin1, "1");

  // Connect outputs to the N2k senders.
  // EDIT: Make sure this matches your tacho configuration above.
  //       Duplicate the lines below to connect more tachos, but be sure to
  //       use different engine instances.
  N2kEngineParameterRapidSender* engine_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 1 Rapid Update", 0,
                                        nmea2000);  // Engine 1, instance 0

  ConfigItem(engine_rapid_sender)
      ->set_title("Engine 1 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 1")
      ->set_sort_order(3015);

  tacho_d1_frequency->connect_to(&(engine_rapid_sender->engine_speed_));

  if (display_present) {
    tacho_d1_frequency->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 3, "RPM D1", 60 * value); }));
  }

  /// BMP280 SENSOR CODE - Engine Room Temp Sensor ////  

  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  // If you need to use the TwoWire library instead of the Wire library, there
  // is a different constructor: see bmp280.h

  bmp280.begin(0x76);

  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* engine_room_temp =
      new RepeatSensor<float>(5000, read_temp_callback);

  auto* engine_room_pressure = 
      new RepeatSensor<float>(60000, read_pressure_callback);

  // Send the temperature to the Signal K server as a Float
  engine_room_temp->connect_to(new SKOutputFloat("propulsion.engineRoom.temperature"));

  engine_room_pressure->connect_to(new SKOutputFloat("propulsion.engineRoom.pressure"));

  ///////////////////////////////////////////////////////////////////
  // Display setup

  // Connect the outputs to the display
  if (display_present) {
    event_loop()->onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });

    // Create a poor man's "christmas tree" display for the alarms
    event_loop()->onRepeat(1000, []() {
      char state_string[5] = {};
      for (int i = 0; i < 4; i++) {
        state_string[i] = alarm_states[i] ? '*' : '_';
      }
      PrintValue(display, 4, "Alarm", state_string);
    });
  }

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }

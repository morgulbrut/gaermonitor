#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <math.h>
#include <TheThingsNetwork.h>
#include <CayenneLPP.h>


#define loraSerial Serial1

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868

// Pin which is connected to the DHT sensor.
#define DHTPIN            4         
// Uncomment the type of sensor in use:
//#define DHTTYPE           DHT11     // DHT 11 
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)

const uint32_t delayMS = 1800000;

// Set your AppEUI and AppKey
const char *appEui = "70B3D57ED00103D7";
const char *appKey = "8DD5A0B1CA11595AE5A00B8EBB2A97AE";

/**
 * from http://wiki.seeedstudio.com/Grove-Temperature_Sensor_V1.2/
 */
const int B = 4275;               // B value of the thermistor
const int R0 = 100000;            // R0 = 100k
const int pinTempSensor = A1;     // Grove - Temperature Sensor connect to A0

DHT_Unified dht(DHTPIN, DHTTYPE);

TheThingsNetwork ttn(loraSerial, Serial, freqPlan);
CayenneLPP lpp(51);

/**
 *  Temperature sensor to get added to the bottle
 */
float get_analog_temperature(){
    int a = analogRead(pinTempSensor);
    float R = 1023.0/a-1.0;
    R = R0*R;
    float temperature = 1.0/(log(R/R0)/B+1/298.15)-273.15; // convert to temperature via datasheet
    Serial.print("Analog temperature = ");
    Serial.println(temperature);
    return temperature;
}

float get_dht_temperature(){
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    Serial.print("DHT temperature: ");
    float temperature = event.temperature;
    Serial.print(temperature);
    Serial.println(" *C");
    return temperature;
  }
   return -99.0;
}

float get_dht_humidity(){
  sensors_event_t event; 
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    Serial.print("DHT humidity: ");
    float humidity = event.relative_humidity;
    Serial.print(humidity);
    Serial.println("%");
    return humidity;
  }
  
   return -99.0;

}

void message(const uint8_t *payload, size_t size, port_t port){
      Serial.println("-- MESSAGE");
  Serial.print("Received " + String(size) + " bytes on port " + String(port) + ":");

  for (int i = 0; i < size; i++)
  {
    Serial.print(" " + String(payload[i]));
  }

  Serial.println();
}

float get_light(){
    int light = analogRead(A0);
    Serial.print("Light: ");
    Serial.println(light);
    return light;
}

void setup() {
  Serial.begin(9600);

  loraSerial.begin(57600);

  ttn.onMessage(message);

  // Wait a maximum of 10s for Serial Monitor
  while (!Serial && millis() < 10000)
    ;
  Serial.println("-- STATUS");
  ttn.showStatus();

  Serial.println("-- JOIN");
  ttn.join(appEui, appKey);

  dht.begin();
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
}

void loop() {  

  Serial.println("-- LOOP");

  lpp.reset();

  bool light = get_light() < 300? false:true;
  Serial.print("Light on: ");
  Serial.println(light);

  lpp.addTemperature(1, get_analog_temperature());
  lpp.addTemperature(2,get_dht_temperature());
  lpp.addRelativeHumidity(3, get_dht_humidity());
  lpp.addDigitalInput(4,light);

  // Send it off
  ttn.sendBytes(lpp.getBuffer(), lpp.getSize());

  //delay(600000);
  delay(10000);
}


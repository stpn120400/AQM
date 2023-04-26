/*
  For Thesis Purposes
  PROPONENTS:
  Absurda, Cristine
  Agustinez, Stephen Rey G.
  Amolato, Glen Rose
  Wendam, Sandra                
*/

//Last UPDATE: April 26, 2023

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "DHT.h"
#include "Adafruit_Sensor.h"
#define RXD2 16  // to sensor TX
#define TXD2 17  // to sensor RX
#define DHT11PIN 18
#define pmsSerial Serial1


DHT dht(DHT11PIN, DHT11);

const char* ssid = "Denzel_HomeNet";
const char* password = "DenzelHomeNet2022";
const char* host = "ustpapm.000webhostapp.com";  // hostname

// REPLACE with your Domain name and URL path or IP address with path
const char* serverName = "https://ustpapm.000webhostapp.com/post-esp-data.php";

// Keep this API Key value to be compatible with the PHP code provided in the project page.
// If you change the apiKeyValue value, the PHP file /post-esp-data.php also needs to have the same key
String apiKeyValue = "tPmAT5Ab3j7F9";

int WiFiTimeoutCounter = 0;

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;

void connect_WiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    WiFiTimeoutCounter++;
    if (WiFiTimeoutCounter >= 60) {  //after 30 seconds timeout - reset board
      reboot();
    }
  }
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void get_DHT11_sensor_data(float* x, float* y) {
  Serial.println();
  Serial.println("-----Getting DHT11 Sensor Data-----");
  //Reading DHT11 Data
  *x = dht.readHumidity();
  *y = dht.readTemperature();

  // Check if any reads failed.
  if (isnan(*y) || isnan(*x)) {
    Serial.println("Failed to read from DHT sensor!");
    *x = 0.00;
    *y = 0;
    Serial.println("FAILED");
    return;
  } else {
    Serial.println("SUCCEED");
    Serial.println();
    Serial.print("Temperature: ");
    Serial.print(*y);
    Serial.print("\t\tHumidity: ");
    Serial.print(*x);
    Serial.println("\n---------------------------------------");
    return;
  }
}

void get_PMS5003_sensor_data(int* a, int* b, int* c) {
  Serial.println();
  Serial.println("-----Getting PMS5003 Sensor Data-----");
  if (readPMSdata(&pmsSerial)) {
    Serial.println("SUCCEED");
    // reading data was successful!
    *a = data.pm10_env;
    *b = data.pm25_env;
    *c = data.pm100_env;
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: ");
    Serial.print(*a);
    Serial.print("\t\tPM 2.5: ");
    Serial.print(*b);
    Serial.print("\t\tPM 10: ");
    Serial.println(*c);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:");
    Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:");
    Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:");
    Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:");
    Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:");
    Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:");
    Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");
    return;
  } else {
    delay(10000);  //10 seconds delay before system restarts
    Serial.println("FAILED");
    reboot();
  }
}

boolean readPMSdata(Stream* s) {
  if (!s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  //debugging
  /*for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();*/


  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void*)&data, (void*)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

// PM25 aqi value
int calcAQI25(int pm25aqi) {

  // Uses formula AQI = ( (pobs - pmin) x (aqimax - aqimin) ) / (pmax - pmin)  + aqimin
  float pmin, pmax, amin, amax;

  if (pm25aqi <= 12) {
    pmin = 0;
    pmax = 12;
    amin = 0;
    amax = 50;
    goto aqicalc;
  } else if (pm25aqi <= 35.5) {
    pmin = 12;
    pmax = 35.5;
    amin = 50;
    amax = 100;
    goto aqicalc;
  } else if (pm25aqi <= 55.5) {
    pmin = 35.5;
    pmax = 55.5;
    amin = 100;
    amax = 150;
    goto aqicalc;
  } else if (pm25aqi <= 150.5) {
    pmin = 55.5;
    pmax = 150.5;
    amin = 150;
    amax = 200;
    goto aqicalc;
  } else if (pm25aqi <= 250.5) {
    pmin = 150.5;
    pmax = 250.5;
    amin = 200;
    amax = 300;
    goto aqicalc;
  } else if (pm25aqi <= 350.5) {
    pmin = 250.5;
    pmax = 350.5;
    amin = 300;
    amax = 400;
    goto aqicalc;
  } else if (pm25aqi <= 500.5) {
    pmin = 350.5;
    pmax = 500.5;
    amin = 400;
    amax = 500;
    goto aqicalc;
  } else {
    return 999;
  }

aqicalc:
  float aqi = (((pm25aqi - pmin) * (amax - amin)) / (pmax - pmin)) + amin;
  return aqi;
}

const char* get_aqi_status(int PM25_aqival) {
  if (PM25_aqival <= 50) {
    return "Good";
  } else if (PM25_aqival >= 51 && PM25_aqival <= 100) {
    return "Moderate";
  } else if (PM25_aqival >= 101 && PM25_aqival <= 150) {
    return "Unhealthy for Sensitive Groups";
  } else if (PM25_aqival >= 151 && PM25_aqival <= 200) {
    return "Unhealthy";
  } else if (PM25_aqival >= 201 && PM25_aqival <= 300) {
    return "Very Unhealthy";
  } else if (PM25_aqival >= 300) {
    return "Hazardous";
  }
}

void setup() {
  //Start Serial Monitor
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Wait to allow sensors to get ready
  delay(2000);

  //start DHT11 Sensor
  dht.begin();

  // Set up UART connection
  pmsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //Connect to Wifi Network
  connect_WiFi();


  Serial.println("\nAll sensors initialized, 5-second warm-up delay before first reading");
  delay(5000);
}

void loop() {
  int PM1 = 0;
  int PM2_5 = 0;
  int PM10 = 0;
  float humidity = 0.00;
  float temperature = 0.00;
  float CO_ppm = 0.00;

  //Reading PMS5003 Data
  get_PMS5003_sensor_data(&PM1, &PM2_5, &PM10);

  //Reading DHT11 Data
  get_DHT11_sensor_data(&humidity, &temperature);

  int PM25_aqival = calcAQI25(PM2_5);
  Serial.print("\nPM2.5 AQI value: ");
  Serial.println(PM25_aqival);

  const char* aqi_status = get_aqi_status(PM25_aqival);
  Serial.print("AQI Status: ");
  Serial.println(aqi_status);
  Serial.println();

  //Check WiFi connection status
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClientSecure* client = new WiFiClientSecure;
    client->setInsecure();  //don't use SSL certificate
    HTTPClient https;

    // Your Domain name with URL path or IP address with path
    https.begin(*client, serverName);

    // Specify content-type header
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");

    // Prepare your HTTP POST request data
    String httpRequestData = "api_key=" + apiKeyValue + "&PM1=" + PM1 + "&PM2_5=" + PM2_5 + "&PM10=" + PM10 + "&temperature=" + temperature + "&humidity=" + humidity + "&CO_ppm=" + CO_ppm + "&PM25_aqival=" + PM25_aqival + "&aqi_status=" + aqi_status + "";
    Serial.print("httpRequestData: ");
    Serial.println(httpRequestData);

    // Send HTTP POST request
    int httpResponseCode = https.POST(httpRequestData);

    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    https.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
  Serial.println();
  Serial.println("closing connection");
  //flush the serial buffer
  while (Serial.available()) {
    Serial.read();
    Serial.end();
    Serial.begin(115200);
  }
  while (pmsSerial.available()) {
    pmsSerial.read();
    pmsSerial.end();
    pmsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  }

  delay(20000);  //20 seconds delay before sending new data
}

void reboot() {
  Serial.println("\nRestarting Device");
  ESP.restart();
}

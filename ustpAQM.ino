/*
  For Thesis Purposes
  PROPONENTS:
  Absurda, Cristine
  Agustinez, Stephen Rey G.
  Amolato, Glen Rose
  Wendam, Sandra                
*/

#include "WiFi.h"
#include "DHT.h"
#define RXD2 16  // to sensor TX
#define TXD2 17  // to sensor RX
#define DHT11PIN 18


DHT dht(DHT11PIN, DHT11);

const char* ssid = "Denzel_HomeNet";
const char* password = "DenzelHomeNet2022";
const char* host = "ustpapm.000webhostapp.com";  // hostname

float CO = 0.00;
const char* aqi_status;
float humidity;
float temperature;
int PM1, PM2_5, PM10;
int PM25_aqival;
String send_Status_Read_DHT11 = "";
String send_Status_Read_PMS5003 = "";


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
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void get_DHT11_sensor_data() {
  Serial.println();
  Serial.println("-----Getting DHT11 Sensor Data-----");

  //Reading DHT11 Data
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  // Check if any reads failed.
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    temperature = 0.00;
    humidity = 0;
    send_Status_Read_DHT11 = "FAILED";
  } else {
    send_Status_Read_DHT11 = "SUCCEED";
  }
}

void get_PMS5003_sensor_data() {
  Serial.println("-----Getting PMS5003 Sensor Data-----");
  if (readPMSdata(&Serial1)) {
    PM1 = data.pm10_env;
    PM2_5 = data.pm25_env;
    PM10 = data.pm100_env;
    PM25_aqival = calcAQI25(PM2_5);
    send_Status_Read_PMS5003 = "SUCCEED";
  } else {
    send_Status_Read_PMS5003 = "FAILED";
  }
}

// PM25 aqi value
int calcAQI25(int pm25) {

  // Uses formula AQI = ( (pobs - pmin) x (aqimax - aqimin) ) / (pmax - pmin)  + aqimin
  float pmin, pmax, amin, amax;

  if (pm25 <= 12) {
    pmin = 0;
    pmax = 12;
    amin = 0;
    amax = 50;
    goto aqicalc;
  } else if (pm25 <= 35.5) {
    pmin = 12;
    pmax = 35.5;
    amin = 50;
    amax = 100;
    goto aqicalc;
  } else if (pm25 <= 55.5) {
    pmin = 35.5;
    pmax = 55.5;
    amin = 100;
    amax = 150;
    goto aqicalc;
  } else if (pm25 <= 150.5) {
    pmin = 55.5;
    pmax = 150.5;
    amin = 150;
    amax = 200;
    goto aqicalc;
  } else if (pm25 <= 250.5) {
    pmin = 150.5;
    pmax = 250.5;
    amin = 200;
    amax = 300;
    goto aqicalc;
  } else if (pm25 <= 350.5) {
    pmin = 250.5;
    pmax = 350.5;
    amin = 300;
    amax = 400;
    goto aqicalc;
  } else if (pm25 <= 500.5) {
    pmin = 350.5;
    pmax = 500.5;
    amin = 400;
    amax = 500;
    goto aqicalc;
  } else {
    return 999;
  }

aqicalc:
  float aqi = (((pm25 - pmin) * (amax - amin)) / (pmax - pmin)) + amin;
  return aqi;
}

void get_aqi_status() {
  if (PM25_aqival <= 50) {
    aqi_status = "Good";
  } else if (PM25_aqival >= 51 && PM25_aqival <= 100) {
    aqi_status = "Moderate";
  } else if (PM25_aqival >= 101 && PM25_aqival <= 150) {
    aqi_status = "Unhealthy for Sensitive Groups";
  } else if (PM25_aqival >= 151 && PM25_aqival <= 200) {
    aqi_status = "Unhealthys";
  } else if (PM25_aqival >= 201 && PM25_aqival <= 300) {
    aqi_status = "Very Unhealthy";
  } else if (PM25_aqival >= 300) {
    aqi_status = "Hazardous";
  }
}

void setup() {
  //Start Serial Monitor
  Serial.begin(115200);

  //start DHT11 Sensor
  dht.begin();
  //start PMS5003 Sensor
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //Connect to Wifi Network
  connect_WiFi();
}

void loop() {
  //Reading DHT11 Data
  get_DHT11_sensor_data();
  //Reading PMS5003 Data
  get_PMS5003_sensor_data();

  //Reading data was successful! Print Data's
  //Output PMS5003 Data
  Serial.println();
  Serial.println("\n-------------------------------------");
  Serial.println("Concentration Units (environmental)");
  Serial.print("PM 1.0: ");
  Serial.print(PM1);
  Serial.print("\t\tPM 2.5: ");
  Serial.print(PM2_5);
  Serial.print("\t\tPM 10: ");
  Serial.print(PM10);
  Serial.println("\n-------------------------------------");
  Serial.print("Particules > 0.3 um / 0.1L air:");
  Serial.println(data.particles_03um);
  Serial.print("Particules > 0.5 um / 0.1L air:");
  Serial.println(data.particles_05um);
  Serial.print("Particules > 1.0 um / 0.1L air:");
  Serial.println(data.particles_10um);
  Serial.print("Particules > 2.5 um / 0.1L air:");
  Serial.println(data.particles_25um);
  Serial.print("Particules > 5.0 um / 0.1L air:");
  Serial.println(data.particles_50um);
  Serial.print("Particules > 10.0 um / 0.1L air:");
  Serial.println(data.particles_100um);
  Serial.println("--------------------------------------");
  //Output DHT11 Data
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("ºC ");
  Serial.print("Humidity: ");
  Serial.println(humidity);
  //Get CO Data
  Serial.print("CO: ");
  Serial.print(CO);
  //Get PM2.5 AQI value
  get_aqi_status();
  Serial.print("\nPM2.5 AQI value: ");
  Serial.println(PM25_aqival);
  //Get AQI Status
  get_aqi_status();
  Serial.print("AQI Status: ");
  Serial.println(aqi_status);
  Serial.println();

  Serial.print("connecting to ");
  Serial.println(host);

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }

  // This will send the request to the server
  client.print(String("GET http://ustpapm.000webhostapp.com/api/connect.php?") + ("&PM1=") + PM1 + ("&PM2_5=") + PM2_5 + ("&PM10=") + PM10 + ("&temperature=") + temperature + ("&humidity=") + humidity + ("&CO=") + CO + ("&PM25_aqival=") + PM25_aqival + ("&aqi_status=") + aqi_status + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 1000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }

  Serial.println();
  Serial.println("closing connection");

  delay(15000);
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

  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */

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

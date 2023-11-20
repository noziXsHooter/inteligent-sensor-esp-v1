#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "functions.h"

const char* ssid = "ssid";
const char* password = "pasword";
const char* mqtt_server = "test.mosquitto.org";
const char* mqtt_hivemq_server = "id.s2.eu.hivemq.cloud";
const char* mqtt_hivemq_username = "username";
const char* mqtt_hivemq_pass = "userpassword";
const char* mqtt_hivemq_credentials[2] = {mqtt_hivemq_username, mqtt_hivemq_pass};
const int mqtt_port = 8883;

const char* topics[2] = {
  "home/sensor/passing/state",
  "home/test"
};


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "a.ntp.br", -10800, 60000);

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 3600000 / 60 / 3; // 1 hora em milissegundos


WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

#define PIR D2 //PIR MOTION SENSOR GPIO

/****** root certificate *********/

static const char *root_ca PROGMEM = R"EOF(
  -----BEGIN CERTIFICATE-----
  MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
  TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
  cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
  WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
  MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
  h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
  0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
  A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+UC
  B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
  KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
  OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
  qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
  rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
  HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
  hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
  ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
  3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
  NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
  ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
  TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
  jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
  oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
  4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
  mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
  emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
  -----END CERTIFICATE-----
  )EOF";

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  timeClient.begin();
  //timeClient.setTimeOffset(3600); // Fuso horário (em segundos)
}
/*
  void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
    // it is active low on the ESP-01)
    // but actually the LED is on; this is because
  } else {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
  }

  }
*/
/***** Call back Method for Receiving MQTT messages and Switching LED ****/

void callback(char* topic, byte* payload, unsigned int length) {
  String incommingMessage = "";
  for (int i = 0; i < length; i++) incommingMessage += (char)payload[i];

  Serial.println("Message arrived [" + String(topic) + "]" + incommingMessage);

  //--- check the incomming message
  if ( strcmp(topic, "home/sensor/passing/state") == 0) {
    if (incommingMessage.equals("1")) Serial.println("Message ");  // Turn the LED on
    else digitalWrite(BUILTIN_LED, LOW);  // Turn the LED off
  }
  while (incommingMessage.equals("1")) {
    Serial.println("Ligado!");
    delay(3000);
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_hivemq_username, mqtt_hivemq_pass)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("home/sensor/passing/message", "Laser Passing Connected");
      // ... and resubscribe
      for (int i = 0; i < sizeof(topics) / sizeof(topics[0]); i++) {
        client.subscribe(topics[i]);
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  while (!Serial) delay(1);
  setup_wifi();

#ifdef ESP8266
  espClient.setInsecure();
#else
  espClient.setCACert(root_ca);      // enable this line and the the "certificate" code for secure connection
#endif

  client.setServer(mqtt_hivemq_server, mqtt_port);
  client.setCallback(callback);
}

/**** Method for Publishing MQTT Messages **********/
void publishMessage(const char* topic, String payload , boolean retained) {
  if (client.publish(topic, payload.c_str(), true))
    Serial.println("Message publised [" + String(topic) + "]: " + payload);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  unsigned long minutos = now / 60000;
  long statePIR = digitalRead(PIR);

  DynamicJsonDocument doc(1024);
  doc["deviceId"] = "LP-01";
  //doc["message"] = "Última passagem foi ha " + String(minutos) + " minutos atras.";
  doc["message"] = "Algo foi detectado no sensor de movimento";
  doc["dateTime"] = "";

  char mqtt_message[128];

  int adition = add(adition, 1);
  /*
    if (now - lastMsg > 1000 * 60 * 1) {
      publishMessage("home/sensor/passing", mqtt_message, true);
      lastMsg = now;
      ++value;
      Serial.println("Printando: " + String(adition));
      //snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
      //Serial.print("Publish message: ");
      //Serial.println(msg);
      //Serial.println("{'Passed!': 18-10-2023 - 20:42:20'}");
     // client.publish("home/sensor/passing", "{'Passed!': 18-10-2023 - 20:42:20'}");
    }
  */
  unsigned long currentMillis = millis();
  char dayWeek [7][12] = {"Domingo", "Segunda", "Terca", "Quarta", "Quinta", "Sexta", "Sabado"};

  /*   if (currentMillis - lastUpdate >= updateInterval) {
      lastUpdate = currentMillis;
      timeClient.update();
      String dateTime = String(dayWeek[timeClient.getDay()]) + "-" + String((timeClient.getEpochTime() / 2629743) % 12 + 1) + "-" + String(1970 + (timeClient.getEpochTime() / 31536000));
      // Imprimir a data e hora atualizadas
      Serial.println("Data e Hora Atualizadas: " + dateTime + " - " + timeClient.getFormattedTime());
      doc["dateTime"] = dateTime;
      serializeJson(doc, mqtt_message);
      publishMessage("home/sensor/passing", mqtt_message, true);
    }
  */

  /* TRIGGER PUBLISH MQTT ALERT */
  if (statePIR == HIGH) {
    Serial.println("Motion detected!");
    timeClient.update();
    String dateTime = String(dayWeek[timeClient.getDay()]) + "-" + String((timeClient.getEpochTime() / 2629743) % 12 + 1) + "-" + String(1970 + (timeClient.getEpochTime() / 31536000));
    // Imprimir a data e hora atualizadas
    Serial.println("Data e Hora Atualizadas: " + dateTime + " - " + timeClient.getFormattedTime());
    doc["dateTime"] = dateTime + " - " + timeClient.getFormattedTime();
    serializeJson(doc, mqtt_message);
    publishMessage("home/sensor/passing", mqtt_message, true);
    delay(20000);
  }
  /*     else {
      //  digitalWrite (led, HIGH);
        Serial.println("No motion!");
        delay(1000);
      } */
}

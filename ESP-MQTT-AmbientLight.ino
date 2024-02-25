// Optimize for speed not for size as default
// #pragma GCC optimize("-O3")

// Import required libraries
#include <ESP8266WiFi.h>
#include "PubSubClient.h" // Connect and publish to the MQTT broker

// Uncomment line below for debug
//#define DEBUG

// Replace with your network credentials
constexpr char *ssid = "WIFI_SSID";
constexpr char *password = "WIFI_PASSWORD";

// Mosquitto MQTT server credentials
constexpr char *mqtt_server = "10.0.1.2"; // IP of the MQTT broker
constexpr char *mqtt_username = "JohnDoe";    // MQTT username
constexpr char *mqtt_password = "JohnDoe"; // MQTT password

constexpr char *clientID = "AmbientLights";

// Topics for MQTT to be set as retained in the broker
constexpr char *topic_colorL = "ambient/CL"; // Left lamp Color in HSB format
constexpr char *topic_colorR = "ambient/CR"; // Right lamp Color in HSB format
constexpr char *topic_gain = "ambient/LG";    // Set gain for color sensor from 1 to 60
constexpr char *topic_brightness = "ambient/LB"; // Brightness 0 to 31 as per APA102 datasheet
constexpr char *topic_mode = "ambient/LM";     // 0 = user color, 1 = color from sensor
constexpr char *topic_effect = "ambient/LE";   // 0 = none, 1 = rainbow 2...
constexpr char *topic_lampPower = "ambient/LP"; // LED strip power

constexpr char *topic_volts = "ambient/SV";  // Sensor reading for Volts
constexpr char *topic_amps = "ambient/SI";   // Sensor reading for Current
constexpr char *topic_watts = "ambient/SW";  // Sensor reading for Power
constexpr char *topic_energy = "ambient/SE"; // Sensor reading for Energy

// We need 11 bytes to store the values received
// 6 bytes for colors, 1 byte for gain, 1 byte for brightness, 1 byte for color mode, 1 byte for effect, 1 byte for lamp power
constexpr uint8_t ArrLen = 11;
constexpr uint8_t DMA_Len = 13; // (ArrLen + 2 Markers)
constexpr uint8_t sMarker = 60; // <
constexpr uint8_t eMarker = 62; // >

// Array positions
// 0 to 5 are the 2x 3 bytes for each HSB color
constexpr uint8_t Gain = 6;       // Gain for sensors
constexpr uint8_t Brightness = 7; // APA102 Brightness
constexpr uint8_t Mode = 8;       // Lamp mode
constexpr uint8_t Effect = 9;     // Lamp effect
constexpr uint8_t LedPow = 10;    // Lamp power

constexpr uint8_t msTime = 50;

// Global variables
uint8_t vals[ArrLen];
uint8_t cnt;
uint32_t MsgTime;
bool newData = false;

union byteToFloat
{
    byte b[4];
    float fval;
} myFloat;

// WiFi and MQTT objects
WiFiClient wifi_Client;
PubSubClient mqtt_Client(wifi_Client);

// Show data in serial window
void printArray() {
#ifdef DEBUG
    Serial.println("Vals Array");
    for (cnt = 0; cnt < ArrLen; cnt++) {
        Serial.print("->");
        Serial.print(cnt);
        Serial.print(": ");
        Serial.println(vals[cnt]);
    }
#endif
}

// Raw data for STM board (the actual brain)
void SendToStm() {
    digitalWrite(LED_BUILTIN, LOW);
    // We use markers for start and end of the packet
    uint8_t DMA_Arr[DMA_Len] = {sMarker, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, eMarker};
    // It is 2.4 times faster than loop with optimization flag
    // It is 3 times faster than loop with no optimization flag
    memcpy(DMA_Arr + 1, vals, ArrLen);
#ifdef DEBUG
    Serial.println();
    Serial.println("DMA Array");
    for (cnt = 0; cnt < DMA_Len; cnt++) {
        Serial.print("*>");
        Serial.print(cnt);
        Serial.print(": ");
        Serial.println(DMA_Arr[cnt]);
    }
#endif
    Serial.write(DMA_Arr, DMA_Len);
    delay(25);
    digitalWrite(LED_BUILTIN, HIGH);
    // delay(50);
    // Serial.write(DMA_Arr, DMA_Len);  
}

void ClearBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
}

void publishToMqtt(const char *mqtt_topic, String mqtt_value) {
    if (!mqtt_Client.publish(mqtt_topic, mqtt_value.c_str())) {
        mqtt_Client.connect(clientID, mqtt_username, mqtt_password);
        delay(10);
        mqtt_Client.publish(mqtt_topic, mqtt_value.c_str());
    }
}

static void HSB2RGB(uint8_t pos, uint16_t Hue, uint8_t Sat, uint8_t Bright) {
    uint8_t tmp = 0;
    if (pos) {
        tmp = 3;
    }

    uint16_t h = Hue % 360;
    uint8_t s = Sat * 255 / 100;
    uint8_t v = Bright * 255 / 100;
    if (s == 0) {
        vals[tmp] = vals[tmp + 1] = vals[tmp + 2] = v;
        return;
    }
    uint8_t sector = h / 60;
    uint8_t remainder = (h % 60) * 255 / 60;
    uint8_t p = v * (255 - s) / 255;
    uint8_t q = v * (255 - (s * remainder) / 255) / 255;
    uint8_t t = v * (255 - (s * (255 - remainder)) / 255) / 255;

    switch (sector) {
      case 0:
          vals[tmp] = v;
          vals[tmp + 1] = t;
          vals[tmp + 2] = p;
          break;
      case 1:
          vals[tmp] = q;
          vals[tmp + 1] = v;
          vals[tmp + 2] = p;
          break;
      case 2:
          vals[tmp] = p;
          vals[tmp + 1] = v;
          vals[tmp + 2] = t;
          break;
      case 3:
          vals[tmp] = p;
          vals[tmp + 1] = q;
          vals[tmp + 2] = v;
          break;
      case 4:
          vals[tmp] = t;
          vals[tmp + 1] = p;
          vals[tmp + 2] = v;
          break;
      default:
          vals[tmp] = v;
          vals[tmp + 1] = p;
          vals[tmp + 2] = q;
          break;
    }
}

void HSBtoBytes(uint8_t idxC, String Value) {
#ifdef DEBUG
    Serial.println("HSBtoBytes");
#endif
    uint8_t idx = Value.length() + 1;
    char buff[idx];
    char *tmpVal;
    uint16_t tmpArr[4];
    Value.toCharArray(buff, idx);
    idx = 0;
    tmpVal = strtok(buff, ",");
    while (tmpVal != NULL) {
        tmpArr[idx] = atoi(tmpVal);
#ifdef DEBUG
        Serial.println(tmpArr[idx]);
#endif
        idx++;
        tmpVal = strtok(NULL, ",");
    }
    HSB2RGB(idxC, tmpArr[0], (uint8_t)tmpArr[1], (uint8_t)tmpArr[2]);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.begin(57600);
    delay(msTime);

    IPAddress staticIP(10, 0, 1, 125);
    IPAddress gateway(10, 0, 1, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dns(10, 0, 1, 1);

    WiFi.config(staticIP, subnet, gateway, dns);
    WiFi.mode(WIFI_STA);
    WiFi.hostname(clientID);
    WiFi.begin(ssid, password);

#ifdef DEBUG
    MsgTime = millis();
#endif

    while (WiFi.status() != WL_CONNECTED) {
#ifdef DEBUG
        Serial.println("*");
#endif
        delay(msTime);
    }

#ifdef DEBUG
    Serial.print("Connected to WiFi in ms");
    Serial.println(millis() - MsgTime);
    MsgTime = millis();
#endif

    mqtt_Client.setServer(mqtt_server, 1883);
    mqtt_Client.setCallback(callback);
    ConnectToServer();
    

#ifdef DEBUG
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
#endif

    ClearBuffer();
    delay(msTime);
    SendToStm();
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    if (!mqtt_Client.connected()) {
        while (!mqtt_Client.connected()) {
            ConnectToServer();
            delay(msTime);
        }
    }

    if (newData) {
#ifdef DEBUG
        printArray();
#endif
        SendToStm();
        newData = false;
    }

    if (Serial.available() > 17) {
        uint8_t tmp_arr[18];
        for (cnt = 0; cnt < 18; cnt++) {
            tmp_arr[cnt] = Serial.read();
        }

        if ((tmp_arr[0] == sMarker) && (tmp_arr[17] == eMarker)) {
            float tmpValue;
            myFloat.b[0] = tmp_arr[1];
            myFloat.b[1] = tmp_arr[2];
            myFloat.b[2] = tmp_arr[3];
            myFloat.b[3] = tmp_arr[4];
            tmpValue = myFloat.fval;
            publishToMqtt(topic_volts, String(myFloat.fval, 6));
            myFloat.b[0] = tmp_arr[5];
            myFloat.b[1] = tmp_arr[6];
            myFloat.b[2] = tmp_arr[7];
            myFloat.b[3] = tmp_arr[8];
            publishToMqtt(topic_amps, String(myFloat.fval, 6));
            myFloat.b[0] = tmp_arr[9];
            myFloat.b[1] = tmp_arr[10];
            myFloat.b[2] = tmp_arr[11];
            myFloat.b[3] = tmp_arr[12];
            publishToMqtt(topic_watts, String(myFloat.fval, 6));
            myFloat.b[0] = tmp_arr[13];
            myFloat.b[1] = tmp_arr[14];
            myFloat.b[2] = tmp_arr[15];
            myFloat.b[3] = tmp_arr[16];
            publishToMqtt(topic_energy, String(myFloat.fval, 6));
        } else {
            ClearBuffer();
        }
    }
    mqtt_Client.loop();
}

void ConnectToServer() {
    if (mqtt_Client.connect(clientID, mqtt_username, mqtt_password)) {
        mqtt_Client.subscribe(topic_colorL);
        mqtt_Client.subscribe(topic_colorR);
        mqtt_Client.subscribe(topic_gain);
        mqtt_Client.subscribe(topic_brightness);
        mqtt_Client.subscribe(topic_mode);
        mqtt_Client.subscribe(topic_effect);
        mqtt_Client.subscribe(topic_lampPower);
#ifdef DEBUG
        Serial.print("Connected to MQTT Broker in ms");
        Serial.println(millis() - MsgTime);
#endif
    }
#ifdef DEBUG
    else {
        Serial.println("Connection to MQTT Broker failed...");
        mqtt_Client.state();
    }
#endif
}

void callback(char *topic, byte *payload, unsigned int length) {
#ifdef DEBUG
    Serial.println();
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]\n ");
#endif
    String messageTemp;
    for (int i = 0; i < length; i++) {
#ifdef DEBUG
        Serial.print((char)payload[i]);
#endif
        messageTemp += (char)payload[i];
    }
#ifdef DEBUG
    Serial.println();
#endif
    if (strcmp(topic, topic_colorL) == 0) {
        HSBtoBytes(0, messageTemp);
        newData = true;
    }

    if (strcmp(topic, topic_colorR) == 0) {
        HSBtoBytes(1, messageTemp);
        newData = true;
    }

    if (strcmp(topic, topic_gain) == 0) {
        vals[Gain] = messageTemp.toInt();
        newData = true;
    }

    if (strcmp(topic, topic_brightness) == 0) {
        vals[Brightness] = messageTemp.toInt();
        newData = true;
    }

    if (strcmp(topic, topic_mode) == 0) {
        vals[Mode] = messageTemp.toInt();
        newData = true;
    }

    if (strcmp(topic, topic_effect) == 0) {
        vals[Effect] = messageTemp.toInt();
        newData = true;
    }

    if (strcmp(topic, topic_lampPower) == 0) {
        vals[LedPow] = messageTemp.toInt();
        newData = true;
    }
}

#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

const char* ssid = "MY-SSID";
const char* password = "MY-PASS";

const char* PARAM_MESSAGE = "message";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setup() {

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "Hello, world");
  });

  // Send a GET request to <IP>/get?message=<message>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest * request) {
    String message;
    if (request->hasParam(PARAM_MESSAGE)) {
      message = request->getParam(PARAM_MESSAGE)->value();
    } else {
      message = "No message sent";
    }
    request->send(200, "text/plain", "Hello, GET: " + message);
  });

  server.on("/stop", HTTP_GET, [] (AsyncWebServerRequest * request) {
    Serial.write('S');
    request->send(200, "text/plain", "STOP eseguito");
  });

  server.on("/go", HTTP_GET, [] (AsyncWebServerRequest * request) {
    Serial.write('G');
    request->send(200, "text/plain", "GO eseguito");
  });

  server.on("/back", HTTP_GET, [] (AsyncWebServerRequest * request) {
    Serial.write('B');
    request->send(200, "text/plain", "BACK eseguito");
  });

  server.on("/left", HTTP_GET, [] (AsyncWebServerRequest * request) {
    Serial.write('L');
    request->send(200, "text/plain", "LEFT eseguito");
  });

  server.on("/right", HTTP_GET, [] (AsyncWebServerRequest * request) {
    Serial.write('R');
    request->send(200, "text/plain", "RIGHT eseguito");
  });

    server.on("/rstIMU", HTTP_GET, [] (AsyncWebServerRequest * request) {
    Serial.write('I');
    request->send(200, "text/plain", "RESET IMU eseguito");
  });

  server.on("/test", HTTP_GET, [] (AsyncWebServerRequest * request) {
    Serial.write('T');
    request->send(200, "text/plain", "TEST eseguito");
  });

  // Send a POST request to <IP>/post with a form field message set to <message>
  server.on("/post", HTTP_POST, [](AsyncWebServerRequest * request) {
    String message;
    if (request->hasParam(PARAM_MESSAGE, true)) {
      message = request->getParam(PARAM_MESSAGE, true)->value();
    } else {
      message = "No message sent";
    }
    request->send(200, "text/plain", "Hello, POST: " + message);
  });

  server.onNotFound(notFound);
  server.begin();
}

void loop() {
}

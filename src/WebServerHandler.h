#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

class WebServerHandler {
public:
  explicit WebServerHandler(AsyncWebServer& s);
  void begin();

private:
  AsyncWebServer& server;

  bool isAuthorized(AsyncWebServerRequest* req);
  void sendGz(AsyncWebServerRequest* req, const uint8_t* data, size_t len, const char* mime);
  void handleNetlist(AsyncWebServerRequest* req);
  void handleStatusJson(AsyncWebServerRequest* req);
  void handleConfigGet(AsyncWebServerRequest* req);
  void handleConfigPost(AsyncWebServerRequest* req, const String& body);
  void handleSubmitNetConfig(AsyncWebServerRequest* req);
  void handleNetconfJson(AsyncWebServerRequest* req);
};

const uint8_t* webserialHtml();
size_t webserialHtmlLen();

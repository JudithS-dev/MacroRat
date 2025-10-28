#include "communication_interface.h"

#include "WiFi.h"

#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "error_handler.h"

PubSubClient *globalMqttClient = nullptr;

// hardcoded topics
constexpr char *MQTT_TOPICS[] = {
    "robot/position",
    "robot/wall",
    "robot/cheese",
    "robot/trap",
    "robot/draw",

    // the only topic we're subscribing to
    "robot/command",
};

CommunicationInterface::CommunicationInterface(const char *ssid, const char *password, const char *mqttServer, uint16_t mqttPort)
    : ssid_(ssid),
      password_(password),
      mqttServer_(mqttServer),
      mqttPort_(mqttPort),
      isConnected_(false),
      wifiClient_(),
      mqttClient_(mqttServer, mqttPort, wifiClient_)
{
    Serial.println("Initializing CommunicationInterface...");
    connectToWiFi();
    initializeMQTT();
    Serial.println("CommunicationInterface initialized.");
    log("\n\n============ STARTING INITIALIZATION OF MACRORAT ROBOT CONTROLLER ============");
    log("CommunicationInterface: Initialization complete!");
}

void CommunicationInterface::connectToWiFi()
{
    int tries = 0;
retryAll:
    WiFi.begin(ssid_, password_);
    Serial.print("Trying to connect to WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
        tries++;

        if (tries > 10)
        {
            if (tries > 20)
            {
                fatalError("Couldn't connect to Wifi %s", ssid_);
                return;
            }
            else
            {
                Serial.println("Retrying WiFi connection...");
                WiFi.disconnect();
                delay(2000);
                goto retryAll; // retry connecting to WiFi
            }
        }
    }
    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    isConnected_ = true;
}

void CommunicationInterface::initializeMQTT()
{
    // TODO: this needs access to other classes to execute commands from the MQTT server

    // try to connect to the MQTT server
    int tries = 0;
    while (!mqttClient_.connect(mqttServer_))
    {
        delay(2000);
        if (mqttClient_.connected())
        {
            break;
        }

        tries++;

        if (tries > 5)
        {
            fatalError("Couldn't connect to MQTT server %s", mqttServer_);
        }
    }

    // subscribe to all topics
    mqttClient_.subscribe(MQTT_TOPICS[5]);

    // for now just print the messages to the serial monitor
    mqttClient_.setCallback([this](char *topic, byte *payload, unsigned int length)
    {

        String msg;
        for (unsigned int i = 0; i < length; i++)
        {
            msg += static_cast<char>(payload[i]);
        }

        command_ = msg; // store the command for later use

        log("Received: %s - %s", topic, msg.c_str());
    });

    if (globalMqttClient == nullptr)
    {
        globalMqttClient = &mqttClient_;
    }
}

void CommunicationInterface::sendRatPosition(int16_t x, int16_t y, Direction orientation)
{
    ArduinoJson::JsonDocument doc;
    doc["x"] = x;
    doc["y"] = y;
    doc["orientation"] = static_cast<int>(orientation);
    mqttClient_.publish(MQTT_TOPICS[0], doc.as<String>().c_str());
}

void CommunicationInterface::sendWallData(int16_t x, int16_t y, bool isNorth, bool isWall)
{
    ArduinoJson::JsonDocument doc;
    doc["x"] = x;
    doc["y"] = y;
    doc["side"] = isNorth ? "north" : "west"; // use string for clarity
    doc["type"] = isWall ? "wall" : "empty";
    mqttClient_.publish(MQTT_TOPICS[1], doc.as<String>().c_str());
}

void CommunicationInterface::sendCheese(int16_t x, int16_t y)
{
    ArduinoJson::JsonDocument doc;
    doc["x"] = x;
    doc["y"] = y;
    mqttClient_.publish(MQTT_TOPICS[2], doc.as<String>().c_str());
}

void CommunicationInterface::sendTrap(int16_t x, int16_t y)
{
    ArduinoJson::JsonDocument doc;
    doc["x"] = x;
    doc["y"] = y;
    mqttClient_.publish(MQTT_TOPICS[3], doc.as<String>().c_str());
}

void CommunicationInterface::pathDrawing(bool enable)
{
    ArduinoJson::JsonDocument doc;
    doc["enable"] = enable;
    mqttClient_.publish(MQTT_TOPICS[4], doc.as<String>().c_str());
}

void CommunicationInterface::loop()
{
    if (isConnected_)
    {
        mqttClient_.loop();
    }
    /*else
    {
        connectToWiFi();
        initializeMQTT();
    }*/
}

String CommunicationInterface::getCommand()
{
    if (command_.isEmpty())
    {
        return String();
    }

    String cmd = command_;
    command_.clear(); // clear the command after retrieving it
    return cmd;
}
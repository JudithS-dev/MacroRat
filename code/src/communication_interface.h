#ifndef __COMMUNICATION_INTERFACE_H__
#define __COMMUNICATION_INTERFACE_H__

#include <cstdint>

#include <WiFi.h>
#include <PubSubClient.h>

#include "direction_enum.h" // for Direction

// we use the global mqtt client to log messages
// this is a workaround to avoid passing the client around everywhere
extern PubSubClient *globalMqttClient;

/**
 * This class is responsible for communication between the robot and the
 * webserver.
 * It handles the wifi connection and the communication with the webserver
 * using mqtt.
 */
class CommunicationInterface
{
public:
    /**
     * @brief Constructor for the CommunicationInterface class.
     * @param ssid The SSID of the WiFi network.
     * @param password The password for the WiFi network.
     * @param mqttServer The MQTT server address.
     * @param mqttPort The MQTT server port (default is 1883).
     */
    CommunicationInterface(const char *ssid, const char *password, const char *mqttServer, uint16_t mqttPort = 1883);

    bool sendMessage(const char *topic, const char *message);

    /**
     * @brief Log a message to the MQTT server.
     * @param message The message to log.
     * @param args The arguments to format the message.
     */
    template <typename... Args>
    static void log(const char *message, Args... args)
    {
        if (globalMqttClient != nullptr)
        {
            char buffer[256];
            snprintf(buffer, sizeof(buffer), message, args...);
            globalMqttClient->publish("log", buffer);
        }
    }

    void sendRatPosition(int16_t x, int16_t y, Direction orientation);

    /**
     * @brief Send wall data to the MQTT server.
     * @param x The X coordinate of the wall.
     * @param y The Y coordinate of the wall.
     * @param isNorth True if the wall is to the north, false if it is to the west.
     * @param isWall True if there is a wall, false if there is no wall
     */
    void sendWallData(int16_t x, int16_t y, bool isNorth, bool isWall);

    void sendCheese(int16_t x, int16_t y);
    void sendTrap(int16_t x, int16_t y);

    /**
     * @brief Enable or disable path drawing.
     * @param enable True to enable path drawing, false to disable it.
     */
    void pathDrawing(bool enable);

    void loop();

    String getCommand();

private:
    void connectToWiFi();

    /**
     * @brief Connect to the MQTT server, retries 5 times.
     */
    void initializeMQTT();

private:
    const char *ssid_;       ///< The SSID of the WiFi network.
    const char *password_;   ///< The password for the WiFi network.
    const char *mqttServer_; ///< The MQTT server address.
    const int mqttPort_;     ///< The MQTT server port.
    bool isConnected_;       ///< Flag to check if the connection is established.

    // If a command is received, it gets stored here.
    // the main loop can then ask for the command
    String command_;

    WiFiClient wifiClient_;   ///< WiFi client instance.
    PubSubClient mqttClient_; ///< MQTT client instance.
};

#endif // __COMMUNICATION_INTERFACE_H__
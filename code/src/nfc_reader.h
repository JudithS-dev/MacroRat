/*#ifndef _NFC_MANAGER_H

#define _NFC_MANAGER_H

#include <SPI.h>
#include <MFRC522.h>

// RFID
#define RST_PIN 15
#define SS_PIN  4
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

// Motor UART
HardwareSerial motorSerial(1); // UART1
const byte LEFT_MOTOR = 0x01;
const byte RIGHT_MOTOR = 0x02;

// Geschwindigkeit
const uint16_t NORMAL_SPEED = 500;
const uint16_t ACTION_SPEED = 1000;
const uint16_t EXTREME_SPEED = 2600;

#endif
*/
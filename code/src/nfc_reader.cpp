/*
#include <nfc_reader.h>

void sendMotorCommand(byte device, byte command, uint16_t speed) 
{
  if (speed > 3200) speed = 3200;
  byte low = speed & 0x1F;
  byte high = (speed >> 5) & 0x7F;

  motorSerial.write(0xAA);
  motorSerial.write(device);
  motorSerial.write(command);
  motorSerial.write(low);
  motorSerial.write(high);
}

void driveForward(uint16_t speed) 
{
  sendMotorCommand(LEFT_MOTOR, 0x06, speed);
  sendMotorCommand(RIGHT_MOTOR, 0x05, speed);
}

void stopMotors() 
{
  sendMotorCommand(LEFT_MOTOR, 0x06, 0);
  sendMotorCommand(RIGHT_MOTOR, 0x05, 0);
}

void turnInCircle(uint16_t speed) 
{
  sendMotorCommand(LEFT_MOTOR, 0x06, speed);  
  sendMotorCommand(RIGHT_MOTOR, 0x06, speed); 
}

void turnInCircleMinus(uint16_t speed)
{
  sendMotorCommand(LEFT_MOTOR, 0x05, speed);
  sendMotorCommand(RIGHT_MOTOR, 0x05, speed);
}

void setup() 
{
  Serial.begin(115200);
  SPI.begin(14, 12, 13, 4);  // SCK, MISO, MOSI, SS Pins
  mfrc522.PCD_Init();
  delay(500); // Halbe Sekunde Delay damit der Chip stabil wird

  motorSerial.begin(9600, SERIAL_8N1, 17, 16); // Motor UART

  for (byte i = 0; i < 6; i++) 
  {
    key.keyByte[i] = 0xFF;
  }

  Serial.println("System gestartet. Dauerfahrt vorwärts..."); // Für das testen ohne Motor
  driveForward(NORMAL_SPEED);  // Initiale Fahrt
}

void loop() 
{
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) 
  {
    delay(100);
    return;
  }

  // Block 4 lesen
  byte buffer[18];
  byte size = sizeof(buffer);
  byte block = 4;

  MFRC522::StatusCode status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) 
  {
    Serial.print("Auth fehlgeschlagen: "); // Für Fehlererkennung, falls was mit Karte nicht stimmt
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  status = mfrc522.MIFARE_Read(block, buffer, &size);
  if (status != MFRC522::STATUS_OK) 
  {
    Serial.print("Lesefehler: "); // Für fatale Fehlererkennung, falls alles durch Karte kaputt geht
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  // Inhalt als String
  char text[17];
  memcpy(text, buffer, 16);
  text[16] = '\0';
  String cardContent = String(text);
  cardContent.trim();

  Serial.print("Karteninhalt: ");
  Serial.println(cardContent);

  // Spezialaktionen
  if (cardContent == "Cheese") 
  {
    Serial.println("Aktion: schneller vorwärts");
    driveForward(3000);
    delay(1000);
    //turnInCircle(3000);
    //delay(100);

    //Cheese Command Algorithmus

    // perfekte 90 Grad
    //Serial.println("Aktion: Im Kreis 800");
    //turnInCircle(800);
    //delay(1000);
  } else if (cardContent == "Falle") 
  {
    //Serial.println("Aktion: Im Kreis 2500");
    turnInCircle(2500);
    delay(1000);

    //Falle Command Algorithmus

  } else 
  {
    Serial.println("Unbekannter Inhalt. Keine Aktion.");
  }

  // Zurück zum Standard: langsam geradeaus
  driveForward(NORMAL_SPEED);

  // Karte deaktivieren
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  delay(500);  // kleine Pause vor nächster Erkennung
}


/*
// Karte wird gelesen und je nach Content wird eine Aktion durchgeführt
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN 15
#define SS_PIN  4

MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

HardwareSerial motorSerial(1); // UART1
const byte LEFT_MOTOR = 0x01;
const byte RIGHT_MOTOR = 0x02;

void sendMotorCommand(byte device, byte command, uint16_t speed) {
  if (speed > 3200) speed = 3200;
  byte low = speed & 0x1F;
  byte high = (speed >> 5) & 0x7F;

  motorSerial.write(0xAA);
  motorSerial.write(device);
  motorSerial.write(command);
  motorSerial.write(low);
  motorSerial.write(high);
}

void tagAction(const String& content) {
  if (content == "Cheese") {
    Serial.println("Aktion: Geradeaus fahren"); // Für das testen ohne Motor
    sendMotorCommand(LEFT_MOTOR, 0x06, 2000);
    sendMotorCommand(RIGHT_MOTOR, 0x05, 2000);
    delay(1000);
    sendMotorCommand(LEFT_MOTOR, 0x06, 0);
    sendMotorCommand(RIGHT_MOTOR, 0x05, 0);
  }
  else if (content == "Falle") {
    Serial.println("Aktion: Kreis drehen"); // Für das testen ohne Motor
    sendMotorCommand(LEFT_MOTOR, 0x06, 800);  
    sendMotorCommand(RIGHT_MOTOR, 0x06, 800); 
    delay(3000);
    sendMotorCommand(LEFT_MOTOR, 0x06, 0);
    sendMotorCommand(RIGHT_MOTOR, 0x05, 0);
  }
  else {
    Serial.println("Unbekannter Karteninhalt"); // Falls eine Karte falsch beschrieben wurde
  }
}

void setup() {
  Serial.begin(115200);
  motorSerial.begin(9600, SERIAL_8N1, 17, 16); // Motor UART
  SPI.begin(14, 12, 13, 4); // SCK, MISO, MOSI, SS
  mfrc522.PCD_Init();

  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  Serial.println("System bereit. Karte scannen..."); // Für das testen ohne Motor
}

void loop() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) 
  {
    delay(100);
    return;
  }

  Serial.print("Karte erkannt: UID: "); // Unnötig aber falls man UID überprüfen möchte
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  byte block = 4; // Wichtig, da im Block 4 die geschriebenen Daten stehen
  byte buffer[18];
  byte size = sizeof(buffer);

  MFRC522::StatusCode status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));

  if (status != MFRC522::STATUS_OK) 
  {
    Serial.print("Authentifizierung fehlgeschlagen: ");
    Serial.println(mfrc522.GetStatusCodeName(status)); // GetStatusCodeName eig schlecht, da je nach Fehler das System komplett hängt (Bsp.: The CRC_A does not match)
    return;
  }

  status = mfrc522.MIFARE_Read(block, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Lesefehler: "); 
    Serial.println(mfrc522.GetStatusCodeName(status)); // Hier dasselbe, falls das passieren sollte, nochmal neu hochladen und andere Karte probieren
    return;
  }

  // Inhalt als String extrahieren
  char text[17]; // 16 Zeichen + Nullterminator
  memcpy(text, buffer, 16);
  text[16] = '\0'; // sicherstellen dass es nullterminiert ist sonst Fehler: 'The CRC_A does not match'

  String cardContent = String(text);
  cardContent.trim();  // Leerzeichen/Nullen am Ende entfernen sonst Fehler: 'The CRC_A does not match'

  Serial.print("Inhalt des Blocks 4: "); // Für das testen ohne Motoren
  Serial.println(cardContent);

  tagAction(cardContent);

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  delay(1000);  
}
*/

/* Allgemeine Funktion vom NFC zum Lesen
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN 15
#define SS_PIN  4

MFRC522 mfrc522(SS_PIN, RST_PIN);  // RFID Instanz

MFRC522::MIFARE_Key key;  // Standard Key A

void setup() {
  Serial.begin(115200);
  SPI.begin(14, 12, 13, 4);  // SCK, MISO, MOSI, SS
  mfrc522.PCD_Init();
  Serial.println("RFID RC522 Reader Ready. Warten auf Karte...");

  // Standard-Key vorbereiten (0xFF)
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
}

void loop() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    delay(50);
    return;
  }

  Serial.print("Karte erkannt, UID: ");
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
  }
  Serial.println();

  // Block 4 lesen (erster nutzbarer Block im Sektor 1)
  byte block = 4;
  byte buffer[18];
  byte size = sizeof(buffer);

  // Authentifizieren mit Key A
  MFRC522::StatusCode status = mfrc522.PCD_Authenticate(
    MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Authentifizierung fehlgeschlagen: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  // Block lesen
  status = mfrc522.MIFARE_Read(block, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Lesefehler: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  Serial.print("Inhalt von Block "); Serial.print(block); Serial.print(": ");

  // Als Text ausgeben
  for (byte i = 0; i < 16; i++) {
    Serial.write(buffer[i]);  // als ASCII-Text
  }
  Serial.println();

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  delay(500);
}
*/

/* Funktion damit der NFC die Karte beschreibt
  Grund hierfür ist, das die Handy App eine hohe Chance hat die Karte zu 'corrupten'
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN 15
#define SS_PIN  4

MFRC522 mfrc522(SS_PIN, RST_PIN);  // RFID Instanz
MFRC522::MIFARE_Key key;

void setup() {
  Serial.begin(115200);
  SPI.begin(14, 12, 13, 4);  // SCK, MISO, MOSI, SS
  mfrc522.PCD_Init();
  Serial.println("Warte auf Karte zum Schreiben...");

  // Standard-Key (0xFF FF FF FF FF FF)
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
}

void loop() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    delay(50);
    return;
  }

  Serial.println("Karte erkannt!");

  // Zu beschreibender Text (max. 16 Zeichen)
  byte dataBlock[16] = "Falle";
  // byte dataBlock[16] = "Cheese";

  byte block = 4;  // Block zum Schreiben
  MFRC522::StatusCode status;

  // Authentifizieren
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
                                    block, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Auth fehlgeschlagen: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  // Daten schreiben
  status = mfrc522.MIFARE_Write(block, dataBlock, 16);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Schreiben fehlgeschlagen: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  Serial.println("Daten erfolgreich geschrieben!");

  // Lesen zur Verifikation
  byte readBuffer[18];
  byte size = sizeof(readBuffer);

  status = mfrc522.MIFARE_Read(block, readBuffer, &size);
  if (status == MFRC522::STATUS_OK) {
    Serial.print("Gelesener Inhalt: ");
    for (byte i = 0; i < 16; i++) {
      Serial.write(readBuffer[i]);  // ASCII-Ausgabe
    }
    Serial.println();
  } else {
    Serial.println("Lesen zur Verifikation fehlgeschlagen.");
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  delay(2000);  // Wartezeit bis zur nächsten Karte
}
*/

/* Allgemeine MotorSteuerungsFunktion
  Eher für das testen der Motoren gedacht
#include <Arduino.h>

HardwareSerial motorSerial(1);  // UART1 für Motorsteuerung

const byte LEFT_MOTOR = 0x01;
const byte RIGHT_MOTOR = 0x02;

void sendMotorCommand(byte device, byte command, uilnt16_t speed) {
  if (speed > 3200) speed = 3200;
  byte low = speed & 0x1F;
  byte high = (speed >> 5) & 0x7F;

  motorSerial.write(0xAA);
  motorSerial.write(device);  // z. B. 0x01 oder 0x02
  motorSerial.write(command); // 0x05 = vorwärts
  motorSerial.write(low);
  motorSerial.write(high);
}

void setup() {
  motorSerial.begin(9600, SERIAL_8N1, 17, 16); // TX = GPIO16, RX = GPIO17
  delay(1000);

}

void loop() {
  
  sendMotorCommand(LEFT_MOTOR, 0x06, 1000); // M0 vorwärts mit 1600
  sendMotorCommand(RIGHT_MOTOR, 0x06, 1000); // M2 vorwärts mit 1600

  delay(2000);
  sendMotorCommand(LEFT_MOTOR, 0x06, 0);    // M0 stoppen
  sendMotorCommand(RIGHT_MOTOR, 0x05, 0);    // M2 stoppen

  delay(1000);

}
*/

//Bibliotheken hinzufügen - Bibliotheken erweitern eine Programmiersprache um neue Funktionen
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <CapacitiveSensor.h>

//dient zum an und ausschalten von debug-messages am Serial
const bool debugMode = true;


//festlegen wo die RX/TX Pins am Arduino sind
//TROUBLESHOOTING: Werte oder Kabel miteinander tauschen
static const uint8_t PIN_MP3_TX   = 10; //TX (Transmit) am Arduino geht auf RX (Receive) am DFPLAYER
static const uint8_t PIN_MP3_RX = 11; // Und anders herum

//SoftwareSerial Schnittstelle erstellen
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

// Player-Objekt erstellen
DFRobotDFPlayerMini player;

//Welche Pins sind SensorPins für den Input (const = nicht zur Laufzeit änderbar, braucht weniger speicher)
static const int SensorPinInput1 = 6;
static const int SensorPinInput2 = 13;
static const int SensorPinInput3 = 2;
static const int SensorPinInput4 = 3;
static const int SensorPinInput5 = 12;
static const int SensorPinInput6 = 5;
static const int SensorPinInput7 = 4;
static const int SensorPinInput8 = 0;
static const int SensorPinInput9 = 8;
static const int SensorPinInput10 = 9;




//Welcher Pin ist der SensorPin für Output (Wo der Widerstand dran ist)
static const int SensorPinOutput = 7;

//Erzeuge SensorObjekte mit den oben definierten Pins
CapacitiveSensor CapSensor1 = CapacitiveSensor(SensorPinOutput, SensorPinInput1);
CapacitiveSensor CapSensor2 = CapacitiveSensor(SensorPinOutput, SensorPinInput2);
CapacitiveSensor CapSensor3 = CapacitiveSensor(SensorPinOutput, SensorPinInput3);
CapacitiveSensor CapSensor4 = CapacitiveSensor(SensorPinOutput, SensorPinInput4);
CapacitiveSensor CapSensor5 = CapacitiveSensor(SensorPinOutput, SensorPinInput5);
CapacitiveSensor CapSensor6 = CapacitiveSensor(SensorPinOutput, SensorPinInput6);
CapacitiveSensor CapSensor7 = CapacitiveSensor(SensorPinOutput, SensorPinInput7);
CapacitiveSensor CapSensor8 = CapacitiveSensor(SensorPinOutput, SensorPinInput8);
CapacitiveSensor CapSensor9 = CapacitiveSensor(SensorPinOutput, SensorPinInput9);
CapacitiveSensor CapSensor10 = CapacitiveSensor(SensorPinOutput, SensorPinInput10);


//pin, an welchen das potentiometer angeschlossen wird
static const int VolumePin = A0;

//pin zum lesen des kopfhörerausgangs
static const int HeadphonePin = A2;

//grenzwert zur ermittlung des Mikrofonstatus
static const int HeadphoneThreshold = 1000;

//pin an dem das Relais zur Steuerung des Lautsprecheroutputs angeschlossen wird
static const int RelayPin = A5;

//diese variable speichert den lautstärkewert falls er zb über einen drehregler änderbar sein soll
int volume = 30;

//globale sensitivity variable für capacitive sensors
int sensitivity = 10;

//globale grenzwert variable für das auslösen des abspielens
int threshold = 1000;

void setup() {

  //Normaler Serial-Port für Kommunikation von Arduino und Computer
  //Diese (Baud)Zahl muss beim Serial-Monitor angegeben werden, um Debug-Messages zu sehen
  Serial.begin(115200);
  //Printe Willkommensnachricht
  Serial.println("System Startet, Gruesse von Moritz");


  //Virtueller Serialport für Kommunikation von Arduino und DFPLAYER
  softwareSerial.begin(9600);



  //Kommunikation zwischen Arduino und DFPLAYER wird gestartet

  if (player.begin(softwareSerial)) {
    //wenn kein Fehler auftritt
    Serial.println("DFPLAYER wurde gefunden, es kann losgehen!");

    player.volume(volume);

  } else {
    //wenn ein fehler auftritt
    Serial.println("!!!Fehler beim Verbinden von Arduino und DFPLAYER!!!");
    Serial.println("Ist die SD-Karte eingelegt?");
    Serial.println("Tausche die RX/TX Kabel!?");
    Serial.println("Programm wird gestoppt - Fehler beheben und Resetten");
    while (1) {};
  }
  if (debugMode) {
    player.play(1);
  }

  //startet und eicht den kapazitiven Sensor
  CapSensor1.set_CS_AutocaL_Millis(0xFFFFFFFF);

  //Setze den RelayPin als Output pin um das Relais steuern zu können
  pinMode(RelayPin, OUTPUT);

  //setze relay auf "an"
  digitalWrite(RelayPin, HIGH);
}

void loop() {
  //Lese den Widerstand des Potentiometers und formatiere die werte in den bereich 0-30
  int nVol = map(analogRead(VolumePin), 0, 1024, 0, 30);
  if (debugMode) {
    Serial.println("New Volume " + nVol);
  }
  //wenn sich der nvol wert von volume unterscheidet, setze neue volume (reduziert aufrufe von volume und hält so software serial frei)
  if (nVol != volume) {
    //stelle volume-wert auf neuen wert.
    volume = nVol;
    //setze lautstärke am dfplayer
    player.volume(volume);
  }

  //lese headphonepin - wenn der headphonepin eine spannung unter dem grenzwert erkennt ist ein Kopfhörer angeschlossen
  int headphoneStatus = analogRead(HeadphonePin);
  //Wenn der kopfhörer eingesteckt ist ... 
  if (headphoneStatus < HeadphoneThreshold) {
    //... schalte Relay aus - Unterbreche Verbindung des Lautsprechers 
      digitalWrite(RelayPin, LOW);
    
    //Schreibe info auf Serial wenn im DebugMode
    if (debugMode) {
      Serial.println("Kopfhörer Verbunden, Wert:");
    }
  }
  //Wenn kein Kophörer eingesteckt ist ... 
  else {
    //schalte Relay ein - Lautsprecher ist verbunden
      digitalWrite(RelayPin, HIGH);
    //Schreibe info auf Serial wenn im debugmode 
    if (debugMode) {
      Serial.println("Kopfhörer NICHT Verbunden, Wert:");
    }
  }

  //Prüft den zustand des Kapazitiven Sensors - sensitivity ist ein wert zur Änderung der Empfindlichkeit
  long result =  CapSensor1.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 1 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    player.play(1);
  }

  result =  CapSensor2.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 2 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    player.play(2);
  }

  result =  CapSensor3.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 3 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    player.play(1);
  }
  result =  CapSensor4.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 4 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    player.play(2);
  }




  //    result =  CapSensor5.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 5 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    player.play(5);
  //  }
  //    result =  CapSensor6.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 6 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    player.play(2);
  //  }
  //    result =  CapSensor7.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 7 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    player.play(7);
  //  }
  //    result =  CapSensor8.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 8 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    player.play(8);
  //  }
  //    result =  CapSensor9.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 9 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    player.play(9);
  //  }
  //    result =  CapSensor10.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 10 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    player.play(10);
  //  }
  delay(10);
}

//Bibliotheken hinzufügen - Bibliotheken erweitern eine Programmiersprache um neue Funktionen
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <CapacitiveSensor.h>

//dient zum an und ausschalten von debug-messages am Serial
const bool debugMode = false;


//festlegen wo die RX/TX Pins am Arduino sind
//TROUBLESHOOTING: Werte oder Kabel miteinander tauschen
static const uint8_t PIN_MP3_TX   = 10; //TX (Transmit) am Arduino geht auf RX (Receive) am DFPLAYER
static const uint8_t PIN_MP3_RX = 11; // Und anders herum

//SoftwareSerial Schnittstelle erstellen
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

// Player-Objekt erstellen
DFRobotDFPlayerMini player;

//Welcher Pin ist der eine SensorPin für Input (const = nicht zur Laufzeit änderbar, braucht weniger speicher)
static const int SensorPinInput1 = 6;

//Welcher Pin ist der andere SensorPin für Input
static const int SensorPinInput2 = 5;

//Welcher Pin ist der SensorPin für Output (Wo der Widerstand dran ist)
static const int SensorPinOutput = 7;

//Erzeuge SensorObjekte mit den oben definierten Pins
CapacitiveSensor CapSensor1 = CapacitiveSensor(SensorPinOutput, SensorPinInput1);
CapacitiveSensor CapSensor2 = CapacitiveSensor(SensorPinOutput, SensorPinInput2);

//pin, an welchen das potentiometer angeschlossen wird
static const int VolumePin = A0;

//diese variable speichert den lautstärkewert falls er zb über einen drehregler änderbar sein soll
int volume = 30;

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

  //startet und eicht den kapazitiven Sensor
  CapSensor1.set_CS_AutocaL_Millis(0xFFFFFFFF);

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


  //Prüft den zustand des Kapazitiven Sensors - 30 ist ein wert zur Änderung der Empfindlichkeit
  long result =  CapSensor1.capacitiveSensor(30);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 1 misst: ");
    Serial.println(result);
  }
  if (result > 1000) {
    player.play(1);
  }

  result =  CapSensor2.capacitiveSensor(30);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 2 misst: ");
    Serial.println(result);
  }
  if (result > 1000) {
    player.play(2);
  }
  delay(10);
}

//Bibliotheken hinzufügen - Bibliotheken erweitern eine Programmiersprache um neue Funktionen

#include "DFRobotDFPlayerMini.h"
#include <CapacitiveSensor.h>

//dient zum an und ausschalten von debug-messages am Serial
const bool debugMode = false;


// Player-Objekt erstellen
DFRobotDFPlayerMini player;

//Welche Pins sind SensorPins für den Input (const = nicht zur Laufzeit änderbar, braucht weniger speicher)
static const int SensorPinInput1 = 40;
static const int SensorPinInput2 = 41;
static const int SensorPinInput3 = 42;
static const int SensorPinInput4 = 36;
static const int SensorPinInput5 = 37;
static const int SensorPinInput6 = 38;
static const int SensorPinInput7 = 39;
static const int SensorPinInput8 = 31;
static const int SensorPinInput9 = 32;
static const int SensorPinInput10 = 33;




//Welcher Pin ist der SensorPin für Output (Wo der Widerstand dran ist)
static const int SensorPinOutputA = 6;
static const int SensorPinOutputB = 7;
static const int SensorPinOutputC = 9;

//Welche Pins zur Steuerung der Volume Sensoren? - Siehe Platine
static const int SensorPinVolUp = 51;
static const int SensorPinVolDown = 52;
static const int SensorPinVolOutput = 5;

//Erzeuge SensorObjekte mit den oben definierten Pins
CapacitiveSensor CapSensor1 = CapacitiveSensor(SensorPinOutputA, SensorPinInput1);
CapacitiveSensor CapSensor2 = CapacitiveSensor(SensorPinOutputA, SensorPinInput2);
CapacitiveSensor CapSensor3 = CapacitiveSensor(SensorPinOutputA, SensorPinInput3);
CapacitiveSensor CapSensor4 = CapacitiveSensor(SensorPinOutputB, SensorPinInput4);
CapacitiveSensor CapSensor5 = CapacitiveSensor(SensorPinOutputB, SensorPinInput5);
CapacitiveSensor CapSensor6 = CapacitiveSensor(SensorPinOutputB, SensorPinInput6);
CapacitiveSensor CapSensor7 = CapacitiveSensor(SensorPinOutputB, SensorPinInput7);
CapacitiveSensor CapSensor8 = CapacitiveSensor(SensorPinOutputC, SensorPinInput8);
CapacitiveSensor CapSensor9 = CapacitiveSensor(SensorPinOutputC, SensorPinInput9);
CapacitiveSensor CapSensor10 = CapacitiveSensor(SensorPinOutputC, SensorPinInput10);

CapacitiveSensor CapSensorVolUp = CapacitiveSensor(SensorPinVolOutput, SensorPinVolUp);
CapacitiveSensor CapSensorVolDown = CapacitiveSensor(SensorPinVolOutput, SensorPinVolDown);



//pin zum lesen des kopfhörerausgangs
static const int HeadphonePin = A2;

//grenzwert zur ermittlung des Mikrofonstatus
static const int HeadphoneThreshold = 10;

//pin an dem das Relais zur Steuerung des Lautsprecheroutputs angeschlossen wird
static const int RelayPin = 8;

//pins für die LEDs
static const int green = 16;
static const int red = 17;
static const int blue = 18;
static const int yellow = 19;

//pin für das Auslesen des DFPLAYER-Status
static const int busy = 10;

//diese variable speichert den lautstärkewert falls er zb über einen drehregler änderbar sein soll
int volume = 10;

//globale sensitivity variable für capacitive sensors
int sensitivity = 1;

//globale grenzwert variable für das auslösen des abspielens
int threshold = 200;

//speichert den zuletzt abgespielten track um doppeltes abspielen zu verhindern (-1 = nicht vorhanden, also bei init immer jeder track spielbar)
int lastPlayed = -1;

//speichert den zeitpunkt des letzten tastendruckes
long lastPlayTime = -1;

//wie lange bis ein track neu gestartet werden kann (in ms)
int repeatPlayDelay = 3000;


//speichert den zeitpunkt des letzten tastendruckes für volume
long lastControlTime = -1;

//wie lange bis eine neue volume gesetzt werden kann (debounce)
int repeatControlDelay = 300;


void setup() {
  //initialisiere LEDs
  pinMode(blue, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(green, OUTPUT);
  //gelbe LED an bis verbindung mit dfplayer hergestellt wurde
  digitalWrite(yellow, HIGH);

  //setze Busy- Pin als Input, um den Status des DFplayers lesen zu können
  pinMode(busy, INPUT);

  //Normaler Serial-Port für Kommunikation von Arduino und Computer
  //Diese (Baud)Zahl muss beim Serial-Monitor angegeben werden, um Debug-Messages zu sehen
  Serial.begin(115200);
  //Printe Willkommensnachricht
  Serial.println("System Startet, Gruesse von Moritz");


  //Virtueller Serialport für Kommunikation von Arduino und DFPLAYER
  Serial3.begin(9600);



  //Kommunikation zwischen Arduino und DFPLAYER wird gestartet

  if (player.begin(Serial3)) {
    //wenn kein Fehler auftritt
    Serial.println("DFPLAYER wurde gefunden, es kann losgehen!");

    player.volume(volume);
    //schalte gelb aus, grün an
    digitalWrite(yellow, LOW);
    digitalWrite(green, HIGH);

  } else {
    //wenn ein fehler auftritt
    Serial.println("!!!Fehler beim Verbinden von Arduino und DFPLAYER!!!");
    Serial.println("Ist die SD-Karte eingelegt?");
    Serial.println("Tausche die RX/TX Kabel!?");
    Serial.println("Programm wird gestoppt - Fehler beheben und Resetten");
    //schalte gelb aus, rot an
    digitalWrite(yellow, LOW);
    digitalWrite(red, HIGH);
    while (1) {};
  }

  //startet und eicht die kapazitiven Sensoren
  CapSensor1.set_CS_AutocaL_Millis(0xFFFFFFFF);

  //Setze den RelayPin als Output pin um das Relais steuern zu können
  pinMode(RelayPin, OUTPUT);

  //setze relay auf "an"
  digitalWrite(RelayPin, LOW );
  delay(1000);
  digitalWrite(RelayPin, HIGH);
}

void loop() {
  //lese headphonepin - wenn der headphonepin eine spannung unter dem grenzwert erkennt ist ein Kopfhörer angeschlossen
  int headphoneStatus = analogRead(HeadphonePin);
  //Wenn der kopfhörer eingesteckt ist ...
  if (headphoneStatus < HeadphoneThreshold) {
    //... schalte Relay aus - Unterbreche Verbindung des Lautsprechers
    //schalte blaue LED aus
    digitalWrite(RelayPin, LOW);
    digitalWrite(blue, LOW);

    //Schreibe info auf Serial wenn im DebugMode
    if (debugMode) {
      Serial.println("Kopfhörer Verbunden, Wert:" + String(headphoneStatus) );
    }
  }
  //Wenn kein Kophörer eingesteckt ist ...
  else {
    //schalte Relay ein - Lautsprecher ist verbunden
    //schalte blau an
    digitalWrite(RelayPin, HIGH);
    digitalWrite(blue, HIGH);
    //Schreibe info auf Serial wenn im debugmode
    if (debugMode) {
      Serial.println("Kopfhörer NICHT Verbunden, Wert:" + String(headphoneStatus));
    }
  }

  int busyState = digitalRead(busy);
  if (debugMode) {
    Serial.print("Busy-Pin hat Status: ");
    Serial.println(busyState);
  }

  digitalWrite(yellow, busyState);


  //Prüft den zustand des Kapazitiven Sensors - sensitivity ist ein wert zur Änderung der Empfindlichkeit
  long result =  CapSensor1.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 1 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    playTrackSetData(1);
  }

  result =  CapSensor2.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 2 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    playTrackSetData(2);
  }
  //
  result =  CapSensor3.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 3 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    playTrackSetData(3);
  }
  result =  CapSensor4.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 4 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    playTrackSetData(4);
  }




  result =  CapSensor5.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 5 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    playTrackSetData(5);
  }
  result =  CapSensor6.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 6 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    playTrackSetData(6);
  }
  result =  CapSensor7.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 7 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    playTrackSetData(7);
  }
  result =  CapSensor8.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 8 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    playTrackSetData(8);
  }
  result =  CapSensor9.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 9 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    playTrackSetData(9);
  }
  result =  CapSensor10.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor 10 misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    playTrackSetData(10);
  }
  result =  CapSensorVolUp.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor VolUp misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    volumePlus();
  }

  result =  CapSensorVolDown.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    Serial.print("Sensor VolDown misst: ");
    Serial.println(result);
  }
  if (result > threshold) {
    volumeMinus();
  }
  delay(10);
}
//Funktion zur erhöhung der Lautstärke
void volumePlus() {
  long currentTime = millis();
  if (volume < 30 &&  currentTime > (lastControlTime + repeatControlDelay)) {
    volume++;
    player.volume(volume);
    lastControlTime = currentTime;
  }
}

//Funktion zur Verringerung der Lautstärke
void volumeMinus() {
  long currentTime = millis();
  if (volume > 0 &&  currentTime > (lastControlTime + repeatControlDelay)) {
    volume--;
    player.volume(volume);
    lastControlTime = currentTime;
  }
}
//spielt eine datei und setzt/prüft entsprechende variablen
void playTrackSetData(int num) {
  long currentTime = millis();
  //spiele nur einen track wenn es ein anderer track ist oder seit dem letzten abspielen genug Zeit vergangen ist
  if (num != lastPlayed || currentTime > (lastPlayTime + repeatPlayDelay)) {
    player.play(num);
    //setze variablen neu
    lastPlayed = num;
    lastPlayTime = currentTime;
    Serial.print("Playing track: ");
    Serial.println(num);
  }
  else if (debugMode) {
    Serial.println("Blocked repeat play");
  }



}

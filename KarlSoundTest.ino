//Bibliotheken hinzufügen - Bibliotheken erweitern eine Programmiersprache um neue Funktionen


#include <CapacitiveSensor.h>

//dient zum an und ausschalten von debug-messages am Serial
const bool debugMode = false;


// Player-Objekt erstellen
#include <DFMiniMp3.h>

// forward declare the notify class, just the name
//
class Mp3Notify;

// define a handy type using serial and our notify class
//
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3;

// instance a DfMp3 object,
//
DfMp3 player(Serial3);


class Mp3Notify
{
  public:
    static void PrintlnSourceAction(DfMp3_PlaySources source, const char* action)
    {
      if (source & DfMp3_PlaySources_Sd)
      {
        Serial.print("SD Card, ");
      }
      if (source & DfMp3_PlaySources_Usb)
      {
        Serial.print("USB Disk, ");
      }
      if (source & DfMp3_PlaySources_Flash)
      {
        Serial.print("Flash, ");
      }
      Serial.println(action);
    }
    static void OnError([[maybe_unused]] DfMp3& mp3, uint16_t errorCode)
    {
      // see DfMp3_Error for code meaning
      Serial.println();
      Serial.print("Com Error ");
      Serial.println(errorCode);
    }
    static void OnPlayFinished([[maybe_unused]] DfMp3& mp3, [[maybe_unused]] DfMp3_PlaySources source, uint16_t track)
    {
      Serial.print("Play finished for #");
      Serial.println(track);
    }
    static void OnPlaySourceOnline([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
    {
      PrintlnSourceAction(source, "online");
    }
    static void OnPlaySourceInserted([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
    {
      PrintlnSourceAction(source, "inserted");
    }
    static void OnPlaySourceRemoved([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
    {
      PrintlnSourceAction(source, "removed");
    }
};


//Welche Pins sind SensorPins für den Input (const = nicht zur Laufzeit änderbar, braucht weniger speicher)
static const int SensorPinInput1 = 40;
static const int SensorPinInput2 = 41;
static const int SensorPinInput3 = 42;
static const int SensorPinInput4 = 43;
static const int SensorPinInput5 = 36;
static const int SensorPinInput6 = 37;
static const int SensorPinInput7 = 38;
//static const int SensorPinInput8 = 39;
//static const int SensorPinInput9 = 31;
//static const int SensorPinInput10 = 32;
//static const int SensorPinInput11 = 33;
//static const int SensorPinInput12 = 34;
//static const int SensorPinInput13 = 27;
//static const int SensorPinInput14 = 28;
//static const int SensorPinInput15 = 29;
//static const int SensorPinInput16 = 30;
//static const int SensorPinInput17 = 23;
//static const int SensorPinInput18 = 24;
//static const int SensorPinInput19 = 25;
//static const int SensorPinInput20 = 26;
//static const int SensorPinInput21 = 50;
//static const int SensorPinInput22 = 49;
//static const int SensorPinInput23 = 48;
//static const int SensorPinInput24 = 47;


//Welcher Pin ist der SensorPin für Output (Wo der Widerstand dran ist)
static const int SensorPinOutputA = 6;
static const int SensorPinOutputB = 7;
static const int SensorPinOutputC = 9;
static const int SensorPinOutputD = 2;
static const int SensorPinOutputE = 22;
static const int SensorPinOutputF = 3;

//Welche Pins zur Steuerung der Volume Sensoren? - Siehe Platine
static const int SensorPinVolUp = 51;
static const int SensorPinVolDown = 52;
static const int SensorPinVolOutput = 5;

//Erzeuge SensorObjekte mit den oben definierten Pins
CapacitiveSensor CapSensor1 = CapacitiveSensor(SensorPinOutputA, SensorPinInput1);
CapacitiveSensor CapSensor2 = CapacitiveSensor(SensorPinOutputA, SensorPinInput2);
CapacitiveSensor CapSensor3 = CapacitiveSensor(SensorPinOutputA, SensorPinInput3);
CapacitiveSensor CapSensor4 = CapacitiveSensor(SensorPinOutputA, SensorPinInput4);

CapacitiveSensor CapSensor5 = CapacitiveSensor(SensorPinOutputB, SensorPinInput5);
CapacitiveSensor CapSensor6 = CapacitiveSensor(SensorPinOutputB, SensorPinInput6);
CapacitiveSensor CapSensor7 = CapacitiveSensor(SensorPinOutputB, SensorPinInput7);
//CapacitiveSensor CapSensor8 = CapacitiveSensor(SensorPinOutputB, SensorPinInput8);
//
//CapacitiveSensor CapSensor9 = CapacitiveSensor(SensorPinOutputC, SensorPinInput9);
//CapacitiveSensor CapSensor10 = CapacitiveSensor(SensorPinOutputC, SensorPinInput10);
//CapacitiveSensor CapSensor11 = CapacitiveSensor(SensorPinOutputC, SensorPinInput11);
//CapacitiveSensor CapSensor12 = CapacitiveSensor(SensorPinOutputC, SensorPinInput12);
//
//CapacitiveSensor CapSensor13 = CapacitiveSensor(SensorPinOutputD, SensorPinInput13);
//CapacitiveSensor CapSensor14 = CapacitiveSensor(SensorPinOutputD, SensorPinInput14);
//CapacitiveSensor CapSensor15 = CapacitiveSensor(SensorPinOutputD, SensorPinInput15);
//CapacitiveSensor CapSensor16 = CapacitiveSensor(SensorPinOutputD, SensorPinInput16);
//
//CapacitiveSensor CapSensor17 = CapacitiveSensor(SensorPinOutputE, SensorPinInput17);
//CapacitiveSensor CapSensor18 = CapacitiveSensor(SensorPinOutputE, SensorPinInput18);
//CapacitiveSensor CapSensor19 = CapacitiveSensor(SensorPinOutputE, SensorPinInput19);
//CapacitiveSensor CapSensor20 = CapacitiveSensor(SensorPinOutputE, SensorPinInput20);
//
//CapacitiveSensor CapSensor21 = CapacitiveSensor(SensorPinOutputF, SensorPinInput21);
//CapacitiveSensor CapSensor22 = CapacitiveSensor(SensorPinOutputF, SensorPinInput22);
//CapacitiveSensor CapSensor23 = CapacitiveSensor(SensorPinOutputF, SensorPinInput23);
//CapacitiveSensor CapSensor24 = CapacitiveSensor(SensorPinOutputF, SensorPinInput24);

//definiere sensorobjekte für volume
CapacitiveSensor CapSensorVolUp = CapacitiveSensor(SensorPinVolOutput, SensorPinVolUp);
CapacitiveSensor CapSensorVolDown = CapacitiveSensor(SensorPinVolOutput, SensorPinVolDown);



//pin zum lesen des kopfhörerausgangs
static const int HeadphonePin = A2;

//grenzwert zur ermittlung des Mikrofonstatus
static const int HeadphoneThreshold = 100;

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
int volume = 20;

//globale sensitivity variable für capacitive sensors
int sensitivity = 1;

//globale grenzwert variable für das auslösen des abspielens
int threshold = 20;

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

//beschreibt die länge der liste, welche die kopfhörerwerte speichern soll.
//VORSICHT - Liste mit länge "10" enthält Positionen 0 bis 9
const int headphoneArraySize = 10 ;

//int array enthält letzte Kopfhörer-Werte
int headphoneValues[headphoneArraySize];

//int zum iterieren über das headphoneValue array
int headphoneIterator = 0 ;


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
  player.begin();
  uint16_t count = player.getTotalTrackCount(DfMp3_PlaySource_Sd);
  if (count != 0 ) {
    //wenn kein Fehler auftritt
    Serial.println("DFPLAYER wurde gefunden, es kann losgehen!");

    player.setVolume(volume);
    //schalte gelb aus, grün an
    digitalWrite(yellow, LOW);
    digitalWrite(green, HIGH);
    //player.playGlobalTrack(1);

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
  // digitalWrite(RelayPin, HIGH);
}

void loop() {
  //lese headphonepin - wenn der headphonepin eine spannung unter dem grenzwert erkennt ist ein Kopfhörer angeschlossen
  int headphoneStatus = analogRead(HeadphonePin);
  //speichere den aktuellen wert in der liste
  headphoneValues[headphoneIterator] = headphoneStatus;
  //sorge dafür, dass der wert des nächsten loops an der nächsten arrayposition gespeichert wird
  // - Es sei denn, die Inkrementierung der Variable würde zu einer ungültigen arrayposition zeigen
  if (headphoneIterator + 1 < headphoneArraySize) {
    headphoneIterator++;
  }
  //sonst wird der nächste wert wieder auf position 0 gespeichert
  else {
    headphoneIterator = 0;
  }
  //führe kopfhörercheck durch 
  bool headphoneAnalysis = headphoneArrayCheck();
  if (debugMode){
    Serial.print("Kopfhörer-Analysierer - "); 
    Serial.println(headphoneAnalysis); 
  }
  //Wenn der kopfhörer eingesteckt ist ...
  if (headphoneAnalysis == true) {
    //... schalte Relay aus - Unterbreche Verbindung des Lautsprechers
    //schalte blaue LED aus
    digitalWrite(RelayPin, HIGH);
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
    digitalWrite(RelayPin, LOW);
    digitalWrite(blue, HIGH);
    //Schreibe info auf Serial wenn im debugmode
    if (debugMode) {
      Serial.println("Kopfhörer NICHT Verbunden, Wert:" + String(headphoneStatus));
    }
  }

  int busyState = digitalRead(busy);
  if (debugMode) {
    // Serial.print("Busy-Pin hat Status: ");
    // Serial.println(busyState);
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
  //  result =  CapSensor8.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 8 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(8);
  //  }
  //  result =  CapSensor9.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 9 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(9);
  //  }
  //  result =  CapSensor10.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 10 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(10);
  //  }
  //
  // result =  CapSensor11.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 11 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(11);
  //  }
  //
  //  result =  CapSensor12.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 12 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(12);
  //  }
  //  //
  //  result =  CapSensor13.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 13 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(13);
  //  }
  //  result =  CapSensor14.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 14 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(14);
  //  }
  //
  //
  //
  //
  //  result =  CapSensor15.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 15 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(15);
  //  }
  //  result =  CapSensor16.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 16 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(16);
  //  }
  //  result =  CapSensor17.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 17 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(17);
  //  }
  //  result =  CapSensor18.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 18 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(18);
  //  }
  //  result =  CapSensor19.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 19 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(19);
  //  }
  //  result =  CapSensor20.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 20 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(20);
  //  }
  //result =  CapSensor21.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 21 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(21);
  //  }
  //  result =  CapSensor22.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 22 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(22);
  //  }
  //  result =  CapSensor23.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 23 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(23);
  //  }
  //  result =  CapSensor24.capacitiveSensor(sensitivity);
  //  //Zeige den Messwert des Sensors im Serial Monitor
  //  if (debugMode) {
  //    Serial.print("Sensor 20 misst: ");
  //    Serial.println(result);
  //  }
  //  if (result > threshold) {
  //    playTrackSetData(24);
  //  }
  //
  //
  //


  result =  CapSensorVolUp.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    // Serial.print("Sensor VolUp misst: ");
    // Serial.println(result);
  }
  if (result > threshold) {
    volumePlus();
  }

  result =  CapSensorVolDown.capacitiveSensor(sensitivity);
  //Zeige den Messwert des Sensors im Serial Monitor
  if (debugMode) {
    // Serial.print("Sensor VolDown misst: ");
    //,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, Serial.println(result);
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
    volume += 2;
    player.setVolume(volume);
    lastControlTime = currentTime;
  }
}

//Funktion zur Verringerung der Lautstärke
void volumeMinus() {
  long currentTime = millis();
  if (volume > 0 &&  currentTime > (lastControlTime + repeatControlDelay)) {
    volume -= 2;
    player.setVolume(volume);
    lastControlTime = currentTime;
  }
}
//spielt eine datei und setzt/prüft entsprechende variablen
void playTrackSetData(int num) {
  long currentTime = millis();
  //spiele nur einen track wenn es ein anderer track ist oder seit dem letzten abspielen genug Zeit vergangen ist
  if (num != lastPlayed || currentTime > (lastPlayTime + repeatPlayDelay)) {
    player.playMp3FolderTrack(num);
    //setze variablen neu
    lastPlayed = num;
    lastPlayTime = currentTime;
    Serial.print("Playing track: ");
    Serial.println(num);
  }
  //  else if (num == lastPlayed || currentTime > (lastPlayTime + repeatPlayDelay)) {
  //    player.stop();
  //    //setze variablen neu
  //    lastPlayed = -1;
  //    lastPlayTime = currentTime;
  //    Serial.print("Stopping  ");
  //
  //  }
  //
  else if (debugMode) {
    Serial.println("Blocked repeat play");
  }
}


//prüft wie viele werte des headphoneValues-Arrays auf das Verbunden-Sein eines kopfhörers hindeuten.  
// VORSICHT - wenn das array noch nicht mit echten werten gefüllt ist, kann es zu falschen HeadphoneStatus kommen, aber da 10 loops nur einige MicroSekunden dauern sollten kein Problem 
bool headphoneArrayCheck() {
  int checkNumber = 0;
  for (int i = 0; i < headphoneArraySize; i++) {
    //wenn ein passender wert (entwerder kleiner 100 oder über 1000) gefunden werde inkrementiere checkNumber 
    if (headphoneValues[i] < HeadphoneThreshold || headphoneValues[i] > 1000) {
      checkNumber++; 
    }
  }
  //wenn mehr als die hälfte der Arraypositionen auf den Kopfhörer hindeuten gebe wahr zurück, sonst falsch 
  if (checkNumber > headphoneArraySize/2){
    return true; 
  }
  else {
    return false; 
  }
}

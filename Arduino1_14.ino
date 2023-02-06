// Finale Version für ersten Feldversuch!
// Der Arduino geht in den Power-down Modus, sobald er nichts aktiv tut. Der Watchdog weckt ihn alle 8 Sekunden kurz. 
//   Bei Betätgung des Drucktasters löst Pin 2 ein Interrupt und weckt ihn ebenfalls auf.
// Es wurde ein zusätzliches Relais eingebaut, welches die 5V für die Kamera erst dann freigibt, wenn diese benötigt wird (also bei Drucktaster-Betätigung).
// Der Stromverbrauch ist dadurch von 0,27A auf 0,04A (1/9) reduziert!
// Wenn RTC und(/oder?) SD-Breakout-Board abgesteckt sind, schaltet die Powerbank nach kurzer Zeit (ca. 15min) ab!
// Der Service-Drucktaster trägt Lockstoff auf. Wird vorher der Drucktaster (Wildkatzenerkennung) gedrückt, werden die Motoren bewegt und anschließend Lockstoff aufgetragen 
//  (sinnvoll zum Nagel einfahren!). ACHTUNG: Der 10-Minuten-Timer wird nicht zurückgesetzt, d.h. nach dessen Ablauf wird trotzdem Klebeband und Lockstoff ausgeführt.
// Auf der SD-Karte wird nur eine Logdatei angelegt, in diese werden alle Wildkatzenerkennungen mit Zeitstempel eingetragen. 
// Die geschossenen Fotos werden im Format "JJJJ_MM_DD__hh-mm-ss.JPG" auf der SD-Karte gespeichert.
//
// ACHTUNG: Beim ursprünglichen Aufspielen der Version "Arduino1_14" auf den anderen Arduino (jetzt am Motorshield angeschlossen), 
//  kam es zu ungewöhnlichen Fehlern (hat Kamerarelais im Intervall geschalten)! Daher getauscht, jetzt funktionierts.

// Akkulaufzeit noch unbekannt!



// Bibliotheken
#include <BufferedPrint.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <RingBuf.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>
#include <Adafruit_VC0706.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "RTClib.h"
#include "avr/sleep.h"
#include <avr/power.h>
#include <avr/wdt.h>

//Konstanten
#define CHIPSELECT 10         //f. Busverbindung f. Kamera
#define UHRZEIT_LOCKSTOFF 00  //Gewuenschte Uhrzeit fuer taeglichen Lockstoffauftrag (Stunden)
#define MINUTEN_LOCKSTOFF 2  //Gewuenschte Uhrzeit fuer taeglichen Lockstoffauftrag (Minuten)
#define END_PIN 4             //von Arduino2  auf Arduino1, gibt 12V-Aktionsende bekannt
#define SWITCH_PIN 2          //Drucktaster Wildkatze, muss auf Pin 2 oder 3 liegen wegen Interrupt!
#define WILDCAT_PIN 3         //von Arduino1 auf Arduino2, Wildkatzenmerker
#define SUPPLY_PIN A3         //schaltet Relais für 12V (startet Arduino 2)
#define CAMERA_SUPPLY 5       //Schaltet Relais für 5V Versorgung der Kamera

//Achtung!!! Kamerapins NICHT auf 1 oder 2 definieren!!!
SoftwareSerial cameraconnection(7, 6);    // 7=TX (White), 6=RX (Green)
Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);

// Variablen
String logFileName = "Log-Datei.txt";   // Name der Datei zum Abspeichern der Daten
String dateAndTime = "Datum ungesetzt"; // String fuer Logfile-Eingraege und Bildnamen
int hour_rtc;
int minute_rtc;
int second_rtc;
//int day_s = 1;

volatile bool wdt_interrupt = false;     //watchdog
volatile bool switch_state = false;      //aktueller Status am Drucktaster, false = aktuell nicht gedrueckt
bool end_pin_state = false;              //aktueller Status am  END_PIN 
bool wildcat_pin_state = false;
bool sd_reachable = true;                //speichert, ob SD-Karte vorhanden ist oder nicht
//unsigned long int milliseconds_per_day = 300000;
unsigned long int timestamp = 0;

int counter10minutes = -1;                //Anzhal der 8-Sekundenintervalle bis 10 Minuten um sind
int counter6pm = -1;                      //Anzahl der 8-Sekundenintervalle bis das naechste Mal 18:00 Uhr ist

RTC_DS3231 rtc;
SdFat sd;
SdFile file;
SdFile dirFile;

// Watchdog imer Interrupt Service Routine
ISR(WDT_vect)
{
	wdt_interrupt = true;
}

//Wenn Drucktaster gedrueckt:
void isrSwitchInterrupt()
{
	detachInterrupt(digitalPinToInterrupt(SWITCH_PIN));
	switch_state = true;
}

void enterSleepMode()
{
	attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), isrSwitchInterrupt, LOW);
                                  //ACHTUNG!!!
  switch_state = false;           //WICHTIG an dieser Stelle! SONST WIRD "DRUCKTASTER" IMMER WIEDER ERKANNT!!
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	power_adc_disable();    // disable analog inputs
	power_spi_disable();
	power_timer0_disable();
	power_timer2_disable();
	power_twi_disable();
	sleep_mode();
	sleep_disable();
}

//Wenn Drucktaster betaetigt (Achtung invertierte Logik):
void switch_pressed() {
	Serial.println(F("Drucktaster betätigt!"));
  digitalWrite(CAMERA_SUPPLY, HIGH);                      //Anschalten der Kamera-Versorgungsspannung
  delay(50);                                              //Zeitverzug notwendig fuer Anschalten der Kamera-Versorgungsspannung
	digitalWrite(WILDCAT_PIN, HIGH);                        // Wildkatzen-Pin auf HIGH setzen
	take_picture();												                  // Foto schießen
	timestamp = millis();                                   // Zeitstempel für Aktivierung Arduino 2 setzen
	//counter10minutes = 66;                                  // 10-Minuten-Counter neu auf eig 75 setzen (75*8s = 600s = 10min), aber da Zeitversatz angepasst!
	counter10minutes = 2;                                 // 10-Minuten-Counter auf 2 Intervalle a 8 Sekunden setzen
  digitalWrite(CAMERA_SUPPLY, LOW);                       //Ausschalten der Kamera-Versorgungsspannung
}

void reset6pmCounter() {
	DateTime now = rtc.now();
  DateTime next = DateTime(now.year(), now.month(), now.day() + 1, UHRZEIT_LOCKSTOFF, MINUTEN_LOCKSTOFF, 0);      //für den Folgetag (Normalbetrieb)
	//DateTime next = DateTime(now.year(), now.month(), now.day(), UHRZEIT_LOCKSTOFF, MINUTEN_LOCKSTOFF, 0);        //für den aktuellen Tag (Testbetrieb)
	counter6pm = (next.secondstime() - now.secondstime()) / 8;                                                      //Counter neu setzen
  counter6pm = counter6pm - 14;                                                                                    //Ausgleich Zeitversatz von 2 Minuten
}

void lockstoff() {

  rtc_timeToString();
	//wildcat_pin_state = digitalRead(WILDCAT_PIN);   //ersetzt durch:
  if (digitalRead(WILDCAT_PIN) == HIGH) {
    wildcat_pin_state = true;
    if(sd_reachable == true){
      File logFile = sd.open(logFileName, FILE_WRITE);
      logFile.print(F("Arduino 2 hochgefahren wegen Wildkatze. Zeitpunkt: "));      //Logdatei-Eintrag
      logFile.println(dateAndTime);                            //Logdatei-Eintrag Aktivierungszeit
      logFile.close();
    }
  } 
  else {
    wildcat_pin_state = false;
    if(sd_reachable == true){
      File logFile = sd.open(logFileName, FILE_WRITE);
      logFile.print(F("Arduino 2 hochgefahren wegen täglichem Lockstoffauftrag. Zeitpunkt: "));      //Logdatei-Eintrag
      logFile.println(dateAndTime);                            //Logdatei-Eintrag Aktivierungszeit
      logFile.close();
    }
  }

	Serial.println(F("Arduino 2 gestartet!"));
	digitalWrite(SUPPLY_PIN, HIGH);								// Arduino 2 mit Strom versorgen
	Serial.print(F("WILDCAT_PIN: "));
	Serial.println(wildcat_pin_state);		      // Wildkatzen-Pin ausgeben
	Serial.print(F("END_PIN: "));						  // dauerhafte Abfrage, ob Arduino 2 mit seiner Aktion fertig ist
	Serial.println(digitalRead( END_PIN));
	delay(5000);												        // wichtige Verögerung, damit Arduino 2 keine ungewollten HIGHs beim Starten ausgibt
	unsigned long int securestamp = millis();		// Sicherheits-Zeitstempel setzen

  do{
    //end_pin_state = digitalRead(END_PIN);  //ersetzt durch:
    if (digitalRead(END_PIN) == HIGH){          // Abfrage Status von  END_PIN 
      end_pin_state = true;    
    }
    else{
      end_pin_state = false;
    }
  }
				                  
	// Solange Arduino 2 nicht bestätigt, dass Aktion vollendet ist, oder 5 Minuten (Sicherheit falls 12V Powerbank leer ist) vergangen sind:
	while (end_pin_state == false || securestamp + 300000 <= millis()); 

	digitalWrite(SUPPLY_PIN, LOW);                              // Arduino 2 herunterfahren
	Serial.print(F("Supply Pin: "));
	Serial.println(digitalRead(SUPPLY_PIN));
	Serial.println(F("Arduino 2 heruntergefahren!"));
	digitalWrite(WILDCAT_PIN, LOW);                             // Wildkatzen-Pin LOW setzen
  delay(500);
	counter10minutes = -1;

	if (counter6pm == 0) {
		reset6pmCounter();
	}
}


void check_sd(){
  int wait_seconds = 30;
  for (int i = 0; i <= wait_seconds; i++){
    if (!sd.begin(CHIPSELECT)){
      Serial.println(F("Konnte keine Verbindung zur SD-Karte aufbauen."));
      delay(1000);
      if (i == wait_seconds){
        Serial.println(F("Fehler SD-Karte! Es werden keine Daten gespeichert.")); 
        sd_reachable = false;
      }
    }
    else{
      Serial.println(F("SD-Karte erkannt."));
      sd_reachable = true;
      i = wait_seconds;
    }
  }
}

void rtc_timeToString(){
  //erstellt aus den RTC-Daten einen String, welcher den aktuellen Zeitpunkt enthaelt.
  //Format: "JJJJ_MM_DD__hh-mm-ss"
  char actTime[21];
    
  DateTime now = rtc.now();
  hour_rtc = now.hour();
  minute_rtc = now.minute();
  second_rtc = now.second();

  sprintf(actTime, "%04d_%02d_%02d__%02d-%02d-%02d", (int)now.year(), (int)now.month(), (int)now.day(), (int)now.hour(), (int)now.minute(), (int)now.second());
  dateAndTime = String(actTime);
}

void take_picture() {

  check_sd();
  if (sd_reachable) {
    Serial.print(F("Speichere Kontakt in: "));
    Serial.println(logFileName);
    delay(500);  // Zeitverzug notwendig!
  
    // Log_Datei öffnen + Daten reinschreiben
    File logFile = sd.open(logFileName, FILE_WRITE);            // Oeffne aktuelle Log_Datei
    if (!logFile) {
      Serial.println(F("Konnte die Datei nicht öffnen"));
     delay(500);
    }
    rtc_timeToString();
    logFile.print(F("Wildkatze erkannt, Zeitpunkt: "));
    logFile.println(dateAndTime);
    logFile.close();  // Schliesse die Datei
      // Try to locate the camera
    if (cam.begin()) {
    }
    else {
      Serial.println(F("Es konnte keine Kamera erkannt werden!"));
      return;
    }
    // Set the picture size - you can choose one of 640x480, 320x240 or 160x120 
    cam.setImageSize(VC0706_640x480);        // biggest
    //Zeitverzug notwendig fuer Kamera (min. 250)!
    delay(500);
    if (!cam.takePicture()) {
      Serial.println(F("Es konnte kein Foto geschossen werden!"));
    }
    else {
      Serial.println(F("Foto geschossen, Dateiname:"));
    }
    Serial.println(dateAndTime + ".JPG");
  
    // erstellt das Bild mit dem entsprechenden Zeitstempel als Dateinamen
    // Format: "JJJJ_MM_DD__hh-mm-ss.JPG"
    File imgFile = sd.open(dateAndTime + ".JPG", FILE_WRITE);
  
    // Get the size of the image (frame) taken  
    uint32_t jpglen = cam.frameLength();
    Serial.print(F("Speichere "));
    Serial.print(jpglen, DEC);
    Serial.print(F(" byte in Datei."));
  
    int32_t time = millis();
    pinMode(8, OUTPUT);
    // Read all the data up to # bytes!
    byte wCount = 0; // For counting # of writes
    while (jpglen > 0) {
      // read 32 bytes at a time;
      uint8_t* buffer;
      uint8_t bytesToRead = min((uint32_t)32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
      buffer = cam.readPicture(bytesToRead);
      imgFile.write(buffer, bytesToRead);
      if (++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
        Serial.print(F("."));
        wCount = 0;
      }
      //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
      jpglen -= bytesToRead;
    }
    imgFile.close();
  }
	Serial.println(F("fertig!"));
  delay(500);                         //Zeitversatz noetig fuer serielle Ausgabe
}

void setup() {
	Serial.begin(9600);
	Serial.println(F("Programmstart"));
	pinMode(END_PIN, INPUT);
	pinMode(SWITCH_PIN, INPUT_PULLUP);
	pinMode(WILDCAT_PIN, OUTPUT);
	pinMode(SUPPLY_PIN, OUTPUT);
	digitalWrite(SUPPLY_PIN, LOW);

  pinMode(CAMERA_SUPPLY, OUTPUT);
  digitalWrite(CAMERA_SUPPLY, LOW);

	// setup Watchdog-Timer
	MCUSR &= ~(1 << WDRF); // remove reset flag
	WDTCSR |= (1 << WDCE) | (1 << WDE); // set WDCE, access prescaler
	WDTCSR = (1 << WDP0) | (0 << WDP1) | (0 << WDP2) | (1 << WDP3); // set prescaler bits to to 8s
	//WDTCSR = (1 << WDP0) | (1 << WDP1) | (1 << WDP2); // set prescaler bits to to 2s
	WDTCSR |= 1 << WDIE; // access WDT interrupt

  check_sd();                     //Verbindung SD-Karte checken
	if (!rtc.begin()) {             //Verbindung RTC checken
		Serial.println(F("Keine Verbindung zur Real-Time-Clock!"));
		Serial.flush();
		abort();
	}

	// Falls RTC neu gesetzt werden muss
	// Setzt RTC auf Compilierzeit
	//rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
 
  // Setzt RTC auf 23:45 Uhr
  //rtc.adjust(DateTime(2023, 1, 14, 23, 45, 0));
	reset6pmCounter();
   if (sd_reachable == true){
    File logFile = sd.open(logFileName, FILE_WRITE);
    logFile.print(F("Lockstock aktiviert, Zeitpunkt: "));      //Logdatei-Eintrag
    rtc_timeToString();
    logFile.println(dateAndTime);                           //Logdatei-Eintrag Aktivierungszeit
    logFile.close();
  }
  Serial.println(F("Setup fertig"));
  delay(500);
}

void loop() {
	//alle 8 Sekunden
  //ACHTUNG: Serielle Ausgaben in den Interrupts fuehren zu Fehlern beim Erkennen des Drucktasters!
	if (wdt_interrupt) {
    wdt_interrupt = false;
		if (counter10minutes > 0) {
			counter10minutes--;
		}
		if (counter6pm > 0) {
			counter6pm--;
		}
	}

	if (switch_state) {
		switch_state = false;
		power_all_enable(); // enable everything
		switch_pressed();
	}

	if (counter10minutes == 0 || counter6pm == 0) {         //wenn einer der Counter runtergelaufen ist
		power_all_enable(); // enable everything
		lockstoff();
	}
	enterSleepMode();
}

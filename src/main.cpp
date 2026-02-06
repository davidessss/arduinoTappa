#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ModbusSlave.h>
#include <AHT10.h>
#include <Wire.h>
#include <EEPROM.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"


#define PulsanteSU 6
#define PulsanteGIU 7

#define FINESTRA 4
#define TAPPARELLA 5

#define ReleSU A1
#define ReleGIU A0

#define BEEP A3

#define SENSOREEXT 2
#define SENSOREINT 3

#define SLAVE_ID 2

#define SERIAL_BAUDRATE 9600
#define SERIAL_PORT RS485
#define rxPin 9
#define txPin 8
#define I2C_ADDRESS 0x3C
#define period 5000
#define RST_PIN -1
#define msAntiRimbalzo 200
#define msPulsPremuto 1000
#define LONG_PRESS_THRESHOLD 1000UL
#define holdTime 1000

unsigned long tempoTotale; // ms per corsa completa (da calibrare)
unsigned long startMillisTemp = 0;  //some global variables available anywhere in the program

float Temperature = 0;
float Humidity = 0;

int PercentualeMemo = -1;           // posizione all'avvio (0=giù, 100=su, -1=avvio)
int PercentualeModBus = 0;
bool inMovimento = false;          // qualsiasi movimento in corso (manuale o automatico)
bool direzioneSU = false;
unsigned long startMillisTappa = 0;
unsigned long durata = 0;          // durata prevista per movimento automatico
int PercentualeStart = 0;
int PercentualeTarget = 0;

// gestione pressione
unsigned long pressStartSU = 0;
unsigned long pressStartGIU = 0;
bool premutoSU = false;
bool premutoGIU = false;
bool AllarmeSensoreExt = false;
bool AllarmeSensoreInt = false;
bool AllarmeTapparella = false;

// gestione manual hold (tenere premuto)
bool manualHold = false;
bool manualDirectionSU = false;
unsigned long manualHoldStart = 0;
bool ahtOK = false; // Indica se il sensore è stato inizializzato correttamente
bool DisplayAcceso = true; // Accende o spegne il display


SSD1306AsciiAvrI2c oled;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);
SoftwareSerial RS485 (rxPin, txPin);
Modbus slave(RS485, SLAVE_ID, MODBUS_CONTROL_PIN_NONE);

uint8_t readDigital(uint8_t fc, uint16_t address, uint16_t length, void* data);
uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length, void* data);
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length, void* data);
uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length, void* data);


void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(PulsanteSU, INPUT);
  pinMode(PulsanteGIU, INPUT);
  pinMode(FINESTRA, INPUT);
  pinMode(TAPPARELLA, INPUT);
  pinMode(ReleSU, OUTPUT);
  pinMode(ReleGIU, OUTPUT);
  pinMode(BEEP, OUTPUT);
  pinMode(SENSOREEXT, INPUT);
  pinMode(SENSOREINT, INPUT);
  digitalWrite(ReleSU, LOW);
  digitalWrite(ReleGIU, LOW);
  Serial.begin(9600);
  Serial.println(F("Inizializzo"));
  RS485.begin(SERIAL_BAUDRATE);
    slave.cbVector[CB_READ_COILS] = readDigital;
    slave.cbVector[CB_READ_HOLDING_REGISTERS] = readMemory;
    slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
    slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemory;

    slave.begin(SERIAL_BAUDRATE);
    for (int i = 0; i < 5; i++) {
        if (myAHT20.begin()) {
            ahtOK = true;
            digitalWrite(BEEP, HIGH); // beeppa
            delay(100);
            digitalWrite(BEEP, LOW); // beeppa
            delay(500);
            break;
        } else {
            Serial.println(F("AHT20 non trovato, ritento..."));
            delay(2000);
            digitalWrite(BEEP, HIGH); // beeppa
            delay(300);
            digitalWrite(BEEP, LOW); // beeppa
            delay(500);
        }
    }
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Verdana12);
  oled.set2X();
  oled.clear();
  oled.print(F("ModBus "));
  oled.println(SLAVE_ID);
  oled.println(F("Ver. 1.0"));
  delay(1000);
  oled.clear();

  Serial.println (EEPROM.read(0));
  
  tempoTotale = 35000;
  Serial.println(F("Accensione"));
}

void loop() {

    slave.poll(); 
    Sensore();
    ControlloCicliTapparella();
//
  if (millis() - startMillisTemp >= period)  //test whether the period has elapsed
  {
    Temperature = myAHT20.readTemperature();
    Humidity = myAHT20.readHumidity();
    //oled.clear();

      if (!ahtOK) {
      Serial.println(F("Errore: AHT20 non inizializzato."));
          oled.setCursor(0, 0);
          oled.println(F("AHT20 NC"));
      }else{
          oled.setCursor(40,0);
          oled.print((int)Temperature);
          oled.println(F(" C"));
          oled.setCursor(40,20);
          oled.print((int)Humidity);
          oled.println(F(" %"));
      }
    startMillisTemp = millis();  //IMPORTANT to save the start time of the current LED state.
//Serial.println(freeRam());
  }
  // leggere stato pulsanti e gestire release/press
  gestisciPulsante(PulsanteSU, true);
  gestisciPulsante(PulsanteGIU, false);

  // se il pulsante è premuto e ha superato la soglia, avvia il movimento manuale (una sola volta)
  if (premutoSU && !manualHold) {
    if (millis() - pressStartSU >= LONG_PRESS_THRESHOLD) {
      startManualHold(true);
    }
  }
  if (premutoGIU && !manualHold) {
    if (millis() - pressStartGIU >= LONG_PRESS_THRESHOLD) {
      startManualHold(false);
    }
  }

  // controllo completamento movimento automatico (solo se non è manual hold)
  if (inMovimento && !manualHold) {
    if (durata > 0 && (millis() - startMillisTappa >= durata)) {
      stopMotore();
      inMovimento = false;
      PercentualeMemo = PercentualeTarget;
      Serial.print(F("Arrivato a: "));
      Serial.println(PercentualeMemo);
    }
  }
}

void ControlloCicliTapparella(void) {
  static bool statoPrecedente = LOW;
  static unsigned long ultimoCambio = 0;
  static unsigned int contatoreCicli = 0;

  bool statoAttuale = digitalRead(TAPPARELLA);
  unsigned long ora = millis();

  // Debounce: ignora cambi troppo ravvicinati (<100 ms)
  if (statoAttuale != statoPrecedente && (ora - ultimoCambio) > 100) {
    ultimoCambio = ora;
    statoPrecedente = statoAttuale;
    contatoreCicli++;

    Serial.print(F("Cambio stato n° "));
    Serial.println(contatoreCicli);
  }

  // Se 3 o più cambi → allarme
  if (contatoreCicli >= 6) {  // 6 cambi equivalgono a 3 cicli completi ON/OFF
    AllarmeTapparella = true;
    Serial.println(F("⚠️ Allarme Tapparella: troppi cicli!"));
  }

  // Reset contatore se nessun cambio per troppo tempo (es. 10 s)
  if ((ora - ultimoCambio) > 10000) {
    contatoreCicli = 0;
  }
}


void Sensore(void) {
  static unsigned long pressStart = 0;
  static bool pressed = false;
  
  if (digitalRead(SENSOREEXT) == HIGH) {
    if (!pressed) {
      pressed = true;
      pressStart = millis();
      Serial.print(F("premuto meno di 300 ms"));
    } else if (millis() - pressStart >= holdTime) {
      AllarmeSensoreExt = true;  // Premuto abbastanza a lungo
      Serial.print(F("premuto più di 300 ms"));
    }
  } else if (digitalRead(SENSOREEXT) == LOW){
    pressed = false;
  }
}

void gestisciPulsante(int pin, bool su) {
  bool pressed = (digitalRead(pin) == LOW); // LOW = premuto con INPUT_PULLUP
  if (pressed) {
    // edge di pressione
    if (su && !premutoSU) { pressStartSU = millis(); premutoSU = true; }
    if (!su && !premutoGIU) { pressStartGIU = millis(); premutoGIU = true; }
    // non fare altro qui: il loop() rileva il long-press e avvia il movimento manuale
  } else {
    // edge di rilascio
    if (su && premutoSU) {
      unsigned long durataPress = millis() - pressStartSU;
      premutoSU = false;

      if (manualHold && manualDirectionSU) {
        // eravamo in manual hold up -> ferma e aggiorna posizione
        stopManualHold();
      } else {
        // non era manual hold: è un click breve (o fallback)
        if (durataPress < LONG_PRESS_THRESHOLD) clickBreve(true);
        else clickLungo(true, durataPress); // fallback se qualcosa è andato storto
      }
    }

    if (!su && premutoGIU) {
      unsigned long durataPress = millis() - pressStartGIU;
      premutoGIU = false;

      if (manualHold && !manualDirectionSU) {
        stopManualHold();
      } else {
        if (durataPress < LONG_PRESS_THRESHOLD) clickBreve(false);
        else clickLungo(false, durataPress);
      }
    }
  }
}

void startManualHold(bool su) {
  // avvia il motore mentre l'utente tiene premuto
  manualHold = true;
  manualDirectionSU = su;
  manualHoldStart = millis();
  PercentualeStart = PercentualeMemo;

  // attiva relè corrispondente
  inMovimento = true;
  direzioneSU = su;
  if (su) {
    digitalWrite(ReleSU, HIGH);
    digitalWrite(ReleGIU, LOW);
  } else {
    digitalWrite(ReleGIU, HIGH);
    digitalWrite(ReleSU, LOW);
  }
  Serial.println(su ? "Manuale: inizio pressione SU" : "Manuale: inizio pressione GIU");
}

void stopManualHold() {
  // ferma motore e aggiorna PercentualeMemo in base al tempo di pressione
  unsigned long elapsed = millis() - manualHoldStart;
  int deltaPerc = (int)((elapsed * 100UL) / tempoTotale);
  if (manualDirectionSU) PercentualeMemo = PercentualeStart + deltaPerc;
  else PercentualeMemo = PercentualeStart - deltaPerc;

  if (PercentualeMemo > 100) PercentualeMemo = 100;
  if (PercentualeMemo < 0) PercentualeMemo = 0;

  manualHold = false;
  inMovimento = false;
  stopMotore();
  Serial.print(F("Manuale rilasciato a: "));
  Serial.println(PercentualeMemo);
}

void clickBreve(bool su) {
  if (!inMovimento) {
    // avvio automatico verso fine corsa
    PercentualeStart = PercentualeMemo;
    PercentualeTarget = su ? 100 : 0;
    // evita durata = 0: calcola in modo sicuro
    unsigned long diff = (PercentualeTarget > PercentualeMemo) ? (PercentualeTarget - PercentualeMemo) : (PercentualeMemo - PercentualeTarget);
    if (diff == 0) {
      Serial.println(F("Già alla posizione richiesta"));
      return;
    }
    durata = (unsigned long)tempoTotale * (unsigned long)diff / 100UL;
    startMovimento(su);
    Serial.println(F("Avvio automatico"));
  } else {
    // stop immediato + calcolo nuova posizione (se era automatico)
    aggiornaPercentuale();
    stopMotore();
    inMovimento = false;
    Serial.print(F("Stop anticipato a: "));
    Serial.println(PercentualeMemo);
  }
}

void clickLungo(bool su, unsigned long durataPress) {
  // fallback: se per qualche motivo non si è entrati in manualHold prima del rilascio
  // calcolo la variazione come se fosse stato tenuto premuto
  int deltaPerc = (int)((durataPress * 100UL) / tempoTotale);
  if (su) PercentualeMemo += deltaPerc;
  else PercentualeMemo -= deltaPerc;
  if (PercentualeMemo > 100) PercentualeMemo = 100;
  if (PercentualeMemo < 0) PercentualeMemo = 0;

  stopMotore();
  inMovimento = false;
  manualHold = false;
  Serial.print(F("Manuale (fallback) rilasciato a: "));
  Serial.println(PercentualeMemo);
}

void startMovimento(bool su) {
  // avvio per movimento automatico verso PercentualeTarget
  stopMotore(); // sicurezza
  startMillisTappa = millis();
  inMovimento = true;
  direzioneSU = su;
  if (su) {
    digitalWrite(ReleSU, HIGH);
    digitalWrite(ReleGIU, LOW);
  } else {
    digitalWrite(ReleGIU, HIGH);
    digitalWrite(ReleSU, LOW);
  }
}

void stopMotore() {
  digitalWrite(ReleSU, LOW);
  digitalWrite(ReleGIU, LOW);
  // non azzeriamo inMovimento qui perché viene gestito dai chiamanti (start/stop)
}

void aggiornaPercentuale() {
  // aggiorna PercentualeMemo in base al progresso del movimento automatico
  if (durata == 0) return; // sicurezza
  unsigned long elapsed = millis() - startMillisTappa;
  float progress = (float)elapsed / (float)durata;
  if (progress > 1.0) progress = 1.0;
  int delta = (int)((PercentualeTarget - PercentualeStart) * progress);
  PercentualeMemo = PercentualeStart + delta;
  if (PercentualeMemo < 0) PercentualeMemo = 0;
  if (PercentualeMemo > 100) PercentualeMemo = 100;
}

// Handle the function code Read Holding Registers (FC=03) and write back the values from the EEPROM (holding registers).
uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length, void* data)
{
  for (uint16_t i = 0; i < length; i++) {
    uint16_t reg = address + i;
    int16_t val = 0;

    switch (reg) {
      case 50: val = Temperature * 100; break;
      case 51: val = Humidity * 100;    break;
      case 52: val = PercentualeMemo;  break;
      default:
        val = 0;   // oppure ignora
        break;
    }

    slave.writeRegisterToBuffer(i, val);
  }
  return STATUS_OK;
}
  

// Handle the function codes Read Input Status (FC=01/02) and write back the values from the digital pins (input status).
uint8_t readDigital(uint8_t fc, uint16_t address, uint16_t length, void* data)
{
    //Serial.println("Digital");
    if (address == 10){
      slave.writeCoilToBuffer(0, digitalRead(FINESTRA));
      slave.writeCoilToBuffer(1, digitalRead(TAPPARELLA));
      slave.writeCoilToBuffer(2, AllarmeSensoreExt);
      slave.writeCoilToBuffer(3, AllarmeSensoreInt);
      if (AllarmeSensoreExt){
        AllarmeSensoreExt = false;
      }
    }
    return STATUS_OK;
}


uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length, void* data)
{
    // Check if the requested addresses exist in the array
    if (address > 10)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    if (address == 1){
      
      if (DisplayAcceso){
        Serial.print("display acceso");
        DisplayAcceso = false;
        // Spegne il display
        oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
      }else{
        Serial.print("display spento");
        DisplayAcceso = true;
        // Riaccende il display
        oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
      }
    }else if (address == 2){
      Serial.print("Close");
      digitalWrite(ReleGIU,HIGH);
    }else if (address == 3){
      Serial.print("Close");
      digitalWrite(ReleSU,LOW);
    }else if (address == 4){
      Serial.print("Close");
      digitalWrite(ReleGIU,LOW);
    }

    return STATUS_OK;
    
}

uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length, void* data)
{
    if(address == 20){
      PercentualeModBus = slave.readRegisterFromBuffer(0);
      Serial.print("Tapparelle: ");
      Serial.println(PercentualeModBus);
      vaiAPercentuale(PercentualeModBus);
      Serial.print("tempoTotale: ");
      Serial.println(tempoTotale);
      Serial.print("PercentualeMemo: ");
      Serial.println(PercentualeMemo);
      
    }

    return STATUS_OK;
}

void vaiAPercentuale(int target) {
  // Limiti sicurezza
  if (target < 0) target = 0;
  if (target > 100) target = 100;

  // Se già alla posizione richiesta → non fare nulla
  if (PercentualeMemo == target) {
    Serial.print("Già a: ");
    Serial.println(target);
    return;
  }

  // Imposto variabili di movimento
  PercentualeStart = PercentualeMemo;
  PercentualeTarget = target;

  unsigned long diff = (PercentualeTarget > PercentualeMemo) ? 
                        (PercentualeTarget - PercentualeMemo) : 
                        (PercentualeMemo - PercentualeTarget);

  if (diff == 0) return; // nessun movimento richiesto

  // calcola durata proporzionale alla percentuale
  durata = (unsigned long)tempoTotale * (unsigned long)diff / 100UL;

  // direzione: su se target > posizione attuale
  bool su = (PercentualeTarget > PercentualeMemo);
  startMovimento(su);

  Serial.print("Avvio movimento verso ");
  Serial.print(PercentualeTarget);
  Serial.println("%");
}
//
//extern int __heap_start, *__brkval;
//int freeRam() {
//  int v;
//  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
//}

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ModbusSlave.h>
#include <AHT10.h>
#include <Wire.h>
#include <EEPROM.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

/*
 * Firmware tapparella:
 * - controllo locale da pulsanti
 * - controllo remoto Modbus RTU slave
 * - persistenza EEPROM (posizione, cicli, tempoTotale, slaveId)
 *
 * Riferimento mappa Modbus: docs/MODBUS_MAP.md
 */


#define PulsanteSU 6
#define PulsanteGIU 7

#define FINESTRA 4
#define TAPPARELLA 5

#define ReleSU A1
#define ReleGIU A0

#define BEEP A3

#define SENSOREEXT 2
#define SENSOREINT 3

#define DEFAULT_SLAVE_ID 2

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
#define EEPROM_POS_ADDR 0
#define RELAY_DEADTIME_MS 40
#define DEFAULT_TEMPO_TOTALE_MS 35000UL
// EEPROM layout (byte offsets)
#define EEPROM_MAGIC_ADDR 1
#define EEPROM_CYCLES_ADDR 3
#define EEPROM_TEMPO_ADDR 7
#define EEPROM_SLAVE_ID_ADDR 11
#define EEPROM_MAGIC 0xA55A
#define TEMPO_MIN_MS 5000UL
#define TEMPO_MAX_MS 65535UL
#define CYCLES_SAVE_EVERY 10

unsigned long tempoTotale; // ms per corsa completa (da calibrare)
unsigned long startMillisTemp = 0;  //some global variables available anywhere in the program
uint32_t CicliTapparellaTotali = 0;
uint8_t CicliNonSalvati = 0;
uint8_t SlaveId = DEFAULT_SLAVE_ID;

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

struct PressTracker {
  bool pressed = false;
  bool latched = false;
  unsigned long pressStart = 0;
};

PressTracker sensoreExtTracker;
PressTracker sensoreIntTracker;


SSD1306AsciiAvrI2c oled;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);
SoftwareSerial RS485 (rxPin, txPin);
Modbus slave(RS485, DEFAULT_SLAVE_ID, MODBUS_CONTROL_PIN_NONE);

uint8_t readDigital(uint8_t fc, uint16_t address, uint16_t length, void* data);
uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length, void* data);
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length, void* data);
uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length, void* data);
void gestisciPulsante(int pin, bool su);
void startManualHold(bool su);
void stopManualHold();
void clickBreve(bool su);
void clickLungo(bool su, unsigned long durataPress);
void startMovimento(bool su);
void stopMotore();
void aggiornaPercentuale();
void vaiAPercentuale(int target);
void Sensore(void);
void ControlloCicliTapparella(void);
void salvaPosizioneEEPROM();
void salvaCicliEEPROM();
void salvaTempoTotaleEEPROM();
void salvaSlaveIdEEPROM();
void caricaConfigEEPROM();
void registraCicloTapparella();
void registraCicloSeFinecorsa();
void resetCicliTapparella();
bool coilStateFromData(uint8_t fc, void* data);
void aggiornaAllarmeSensore(uint8_t pin, PressTracker& tracker, bool& alarmFlag, const __FlashStringHelper* label);


void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(PulsanteSU, INPUT_PULLUP);
  pinMode(PulsanteGIU, INPUT_PULLUP);
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
  // Carica i parametri persistiti prima di inizializzare lo stack Modbus.
  caricaConfigEEPROM();
  RS485.begin(SERIAL_BAUDRATE);
    slave.cbVector[CB_READ_COILS] = readDigital;
    slave.cbVector[CB_READ_HOLDING_REGISTERS] = readMemory;
    slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
    slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemory;

    slave.setUnitAddress(SlaveId);
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
  oled.println(SlaveId);
  oled.println(F("Ver. 1.0"));
  delay(1000);
  oled.clear();

  PercentualeMemo = EEPROM.read(EEPROM_POS_ADDR);
  if (PercentualeMemo < 0 || PercentualeMemo > 100) {
    PercentualeMemo = 0;
    EEPROM.update(EEPROM_POS_ADDR, (uint8_t)PercentualeMemo);
  }
  Serial.print(F("Posizione iniziale: "));
  Serial.println(PercentualeMemo);
  Serial.print(F("Cicli totali: "));
  Serial.println(CicliTapparellaTotali);
  Serial.print(F("tempoTotale(ms): "));
  Serial.println(tempoTotale);
  Serial.print(F("SlaveId: "));
  Serial.println(SlaveId);
  Serial.println(F("Accensione"));
}

void loop() {

    slave.poll(); 
    Sensore();
    ControlloCicliTapparella();
//
  if (millis() - startMillisTemp >= period)  //test whether the period has elapsed
  {
    if (!ahtOK) {
      Serial.println(F("Errore: AHT20 non inizializzato."));
      oled.setCursor(0, 0);
      oled.println(F("AHT20 NC"));
    } else {
      Temperature = myAHT20.readTemperature();
      Humidity = myAHT20.readHumidity();
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
      salvaPosizioneEEPROM();
      registraCicloSeFinecorsa();
      Serial.print(F("Arrivato a: "));
      Serial.println(PercentualeMemo);
    }
  }
}

void ControlloCicliTapparella(void) {
  static bool inizializzato = false;
  static bool statoPrecedente = LOW;
  static unsigned long ultimoCambio = 0;
  static unsigned int contatoreCicli = 0;

  bool statoAttuale = digitalRead(TAPPARELLA);
  unsigned long ora = millis();

  if (!inizializzato) {
    statoPrecedente = statoAttuale;
    ultimoCambio = ora;
    inizializzato = true;
  }

  // Debounce: ignora cambi troppo ravvicinati (<100 ms)
  if (statoAttuale != statoPrecedente && (ora - ultimoCambio) > 100) {
    ultimoCambio = ora;
    statoPrecedente = statoAttuale;
    contatoreCicli++;

    Serial.print(F("Cambio stato n. "));
    Serial.println(contatoreCicli);
  }

  // Se 3 o più cambi → allarme
  if (contatoreCicli >= 6) {  // 6 cambi equivalgono a 3 cicli completi ON/OFF
    AllarmeTapparella = true;
    Serial.println(F("Allarme tapparella: troppi cicli"));
  }

  // Reset contatore se nessun cambio per troppo tempo (es. 10 s)
  if ((ora - ultimoCambio) > 10000) {
    contatoreCicli = 0;
  }
}


void Sensore(void) {
  aggiornaAllarmeSensore(SENSOREEXT, sensoreExtTracker, AllarmeSensoreExt, F("EXT"));
  aggiornaAllarmeSensore(SENSOREINT, sensoreIntTracker, AllarmeSensoreInt, F("INT"));
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
  salvaPosizioneEEPROM();
  registraCicloSeFinecorsa();
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
    salvaPosizioneEEPROM();
    registraCicloSeFinecorsa();
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
  salvaPosizioneEEPROM();
  registraCicloSeFinecorsa();
  Serial.print(F("Manuale (fallback) rilasciato a: "));
  Serial.println(PercentualeMemo);
}

void startMovimento(bool su) {
  // avvio per movimento automatico verso PercentualeTarget
  stopMotore(); // sicurezza
  delay(RELAY_DEADTIME_MS);
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

// FC03: lettura holding registers (telemetria + configurazione).
uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length, void* data)
{
  (void)fc;
  (void)data;
  for (uint16_t i = 0; i < length; i++) {
    uint16_t reg = address + i;
    int16_t val = 0;

    switch (reg) {
      // HR50..57: mappa documentata in docs/MODBUS_MAP.md
      case 50: val = Temperature * 100; break;
      case 51: val = Humidity * 100;    break;
      case 52: val = PercentualeMemo;  break;
      case 53: val = (int16_t)(CicliTapparellaTotali & 0xFFFF); break;
      case 54: val = (int16_t)((CicliTapparellaTotali >> 16) & 0xFFFF); break;
      case 55: val = (int16_t)(tempoTotale & 0xFFFF); break;
      case 56: val = (int16_t)((tempoTotale >> 16) & 0xFFFF); break;
      case 57: val = SlaveId; break;
      default:
        val = 0;   // oppure ignora
        break;
    }

    slave.writeRegisterToBuffer(i, val);
  }
  return STATUS_OK;
}
  

// FC01: lettura coils input/allarmi.
// Nota: i flag allarme vengono azzerati dopo la lettura.
uint8_t readDigital(uint8_t fc, uint16_t address, uint16_t length, void* data)
{
    (void)fc;
    (void)data;

    if (address < 10 || (address + length) > 15) {
      return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    for (uint16_t i = 0; i < length; i++) {
      bool coilVal = false;
      switch (address + i) {
        case 10: coilVal = digitalRead(FINESTRA); break;
        case 11: coilVal = digitalRead(TAPPARELLA); break;
        case 12: coilVal = AllarmeSensoreExt; break;
        case 13: coilVal = AllarmeSensoreInt; break;
        case 14: coilVal = AllarmeTapparella; break;
        default: return STATUS_ILLEGAL_DATA_ADDRESS;
      }
      slave.writeCoilToBuffer(i, coilVal);
    }

    AllarmeSensoreExt = false;
    AllarmeSensoreInt = false;
    AllarmeTapparella = false;
    return STATUS_OK;
}


uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length, void* data)
{
    if (length == 0) {
      return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    bool state = coilStateFromData(fc, data);

    switch (address) {
      // Coil 1: display ON/OFF
      case 1:
        DisplayAcceso = state;
        oled.ssd1306WriteCmd(state ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
        Serial.println(state ? F("Display ON") : F("Display OFF"));
        break;
      // Coil 2: comando GIU (true start, false stop)
      case 2:
        if (state) {
          PercentualeStart = PercentualeMemo;
          PercentualeTarget = 0;
          durata = tempoTotale * (unsigned long)(PercentualeMemo) / 100UL;
          manualHold = false;
          startMovimento(false);
        } else {
          if (inMovimento) aggiornaPercentuale();
          stopMotore();
          if (inMovimento) {
            inMovimento = false;
            salvaPosizioneEEPROM();
            registraCicloSeFinecorsa();
          }
        }
        break;
      // Coil 3: comando SU (true start, false stop)
      case 3:
        if (state) {
          PercentualeStart = PercentualeMemo;
          PercentualeTarget = 100;
          durata = tempoTotale * (unsigned long)(100 - PercentualeMemo) / 100UL;
          manualHold = false;
          startMovimento(true);
        } else {
          if (inMovimento) aggiornaPercentuale();
          stopMotore();
          if (inMovimento) {
            inMovimento = false;
            salvaPosizioneEEPROM();
            registraCicloSeFinecorsa();
          }
        }
        break;
      // Coil 4: stop globale (solo su true)
      case 4:
        if (state) {
          if (inMovimento) aggiornaPercentuale();
          stopMotore();
          manualHold = false;
          if (inMovimento) {
            inMovimento = false;
            salvaPosizioneEEPROM();
            registraCicloSeFinecorsa();
          }
        }
        break;
      default:
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    return STATUS_OK;
    
}

uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length, void* data)
{
    (void)fc;
    (void)data;
    if (length == 0) {
      return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    if(address == 20){
      // HR20: target posizione tapparella (% 0..100)
      PercentualeModBus = slave.readRegisterFromBuffer(0);
      Serial.print("Tapparelle: ");
      Serial.println(PercentualeModBus);
      vaiAPercentuale(PercentualeModBus);
      Serial.print("tempoTotale: ");
      Serial.println(tempoTotale);
      Serial.print("PercentualeMemo: ");
      Serial.println(PercentualeMemo);
    } else if (address == 21) {
      // HR21: tempoTotale (ms), persistito in EEPROM
      uint16_t nuovoTempo = slave.readRegisterFromBuffer(0);
      if (nuovoTempo >= TEMPO_MIN_MS && nuovoTempo <= TEMPO_MAX_MS) {
        tempoTotale = nuovoTempo;
        salvaTempoTotaleEEPROM();
        Serial.print(F("Nuovo tempoTotale(ms): "));
        Serial.println(tempoTotale);
      } else {
        return STATUS_ILLEGAL_DATA_VALUE;
      }
    } else if (address == 22) {
      // HR22: reset cicli totali (scrivere 1 per confermare)
      uint16_t cmd = slave.readRegisterFromBuffer(0);
      if (cmd == 1) {
        resetCicliTapparella();
        Serial.println(F("Cicli tapparella azzerati"));
      } else {
        return STATUS_ILLEGAL_DATA_VALUE;
      }
    } else if (address == 23) {
      // HR23: Modbus slave id (1..247), persistito in EEPROM
      uint16_t nuovoSlaveId = slave.readRegisterFromBuffer(0);
      if (nuovoSlaveId >= 1 && nuovoSlaveId <= 247) {
        SlaveId = (uint8_t)nuovoSlaveId;
        salvaSlaveIdEEPROM();
        slave.setUnitAddress(SlaveId);
        Serial.print(F("Nuovo SlaveId: "));
        Serial.println(SlaveId);
      } else {
        return STATUS_ILLEGAL_DATA_VALUE;
      }
    } else {
      return STATUS_ILLEGAL_DATA_ADDRESS;
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

bool coilStateFromData(uint8_t fc, void* data) {
  if (data == nullptr) return false;
  if (fc == FC_WRITE_COIL) {
    return *((bool*)data);
  }
  return (*((uint16_t*)data) != 0);
}

void caricaConfigEEPROM() {
  // Magic header: se assente/invalido, inizializza tutta la configurazione.
  uint16_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC) {
    tempoTotale = DEFAULT_TEMPO_TOTALE_MS;
    CicliTapparellaTotali = 0;
    SlaveId = DEFAULT_SLAVE_ID;
    EEPROM.put(EEPROM_MAGIC_ADDR, (uint16_t)EEPROM_MAGIC);
    salvaTempoTotaleEEPROM();
    salvaCicliEEPROM();
    salvaSlaveIdEEPROM();
    return;
  }

  EEPROM.get(EEPROM_CYCLES_ADDR, CicliTapparellaTotali);
  EEPROM.get(EEPROM_TEMPO_ADDR, tempoTotale);
  SlaveId = EEPROM.read(EEPROM_SLAVE_ID_ADDR);
  if (tempoTotale < TEMPO_MIN_MS || tempoTotale > TEMPO_MAX_MS) {
    tempoTotale = DEFAULT_TEMPO_TOTALE_MS;
    salvaTempoTotaleEEPROM();
  }
  if (SlaveId < 1 || SlaveId > 247) {
    SlaveId = DEFAULT_SLAVE_ID;
    salvaSlaveIdEEPROM();
  }
}

void salvaPosizioneEEPROM() {
  if (PercentualeMemo < 0) PercentualeMemo = 0;
  if (PercentualeMemo > 100) PercentualeMemo = 100;
  EEPROM.update(EEPROM_POS_ADDR, (uint8_t)PercentualeMemo);
}

void salvaCicliEEPROM() {
  EEPROM.put(EEPROM_CYCLES_ADDR, CicliTapparellaTotali);
}

void salvaTempoTotaleEEPROM() {
  EEPROM.put(EEPROM_TEMPO_ADDR, tempoTotale);
}

void salvaSlaveIdEEPROM() {
  EEPROM.update(EEPROM_SLAVE_ID_ADDR, SlaveId);
}

void registraCicloTapparella() {
  // Riduce usura EEPROM: flush cicli ogni CYCLES_SAVE_EVERY.
  CicliTapparellaTotali++;
  CicliNonSalvati++;
  if (CicliNonSalvati >= CYCLES_SAVE_EVERY) {
    salvaCicliEEPROM();
    CicliNonSalvati = 0;
  }
}

void registraCicloSeFinecorsa() {
  // Un ciclo valido è conteggiato solo su arrivo a 0% o 100%.
  if (PercentualeMemo == 0 || PercentualeMemo == 100) {
    registraCicloTapparella();
  }
}

void resetCicliTapparella() {
  CicliTapparellaTotali = 0;
  CicliNonSalvati = 0;
  salvaCicliEEPROM();
}

void aggiornaAllarmeSensore(uint8_t pin, PressTracker& tracker, bool& alarmFlag, const __FlashStringHelper* label) {
  if (digitalRead(pin) == HIGH) {
    if (!tracker.pressed) {
      tracker.pressed = true;
      tracker.latched = false;
      tracker.pressStart = millis();
    } else if (!tracker.latched && (millis() - tracker.pressStart >= holdTime)) {
      tracker.latched = true;
      alarmFlag = true;
      Serial.print(F("Allarme sensore "));
      Serial.println(label);
    }
  } else {
    tracker.pressed = false;
    tracker.latched = false;
  }
}
//
//extern int __heap_start, *__brkval;
//int freeRam() {
//  int v;
//  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
//}

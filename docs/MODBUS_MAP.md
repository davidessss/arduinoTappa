# Modbus Map - Arduino Tapparella

Slave RTU configurabile in EEPROM (`HR23`, default `2`), baudrate `9600`.

## Function Codes

| FC | Direzione | Uso |
|---|---|---|
| 01 | Master -> Slave (Read Coils) | Legge stati digitali/allarmi (`coil 10..17`) |
| 03 | Master -> Slave (Read Holding Registers) | Legge telemetria/config (`HR 50..57`) |
| 05/15 | Master -> Slave (Write Coils) | Comandi rapidi (`coil 1..5`) |
| 06/16 | Master -> Slave (Write Holding Registers) | Comandi parametrici (`HR 20..23`) |

## Coil Read (FC01)

| Coil | Tipo | Significato | Note |
|---|---|---|---|
| 10 | R | Stato `FINESTRA` | `digitalRead(FINESTRA)` |
| 11 | R | Stato `TAPPARELLA` | `digitalRead(TAPPARELLA)` |
| 12 | R | Allarme sensore esterno | Latch, reset esplicito con `coil 5=true` |
| 13 | R | Allarme sensore interno | Latch, reset esplicito con `coil 5=true` |
| 14 | R | Allarme cicli tapparella | Latch, reset esplicito con `coil 5=true` |
| 15 | R | Sensore esterno attivo | Stato realtime |
| 16 | R | Sensore interno attivo | Stato realtime |
| 17 | R | Flag riavvio dispositivo | Latch `true` al boot, reset esplicito con `coil 5=true` |

## Coil Write (FC05/FC15)

| Coil | Tipo | Significato | Azione |
|---|---|---|---|
| 1 | W | Display ON/OFF | `true=ON`, `false=OFF` |
| 2 | W | Comando GIU | `true=start`, `false=stop` |
| 3 | W | Comando SU | `true=start`, `false=stop` |
| 4 | W | STOP globale | Eseguito su `true` |
| 5 | W | Reset esplicito allarmi e boot flag | Eseguito su `true` (`coil 12..14` e `17` a `false`) |

## Holding Register Read (FC03)

| HR | Tipo | Valore | Formato |
|---|---|---|---|
| 50 | R | Temperatura | `int16`, `°C * 100` |
| 51 | R | Umidita | `int16`, `% * 100` |
| 52 | R | Posizione tapparella | `0..100` |
| 53 | R | Cicli totali (LSW) | `uint16` |
| 54 | R | Cicli totali (MSW) | `uint16` |
| 55 | R | `tempoTotale` (LSW) | ms, `uint16` |
| 56 | R | `tempoTotale` (MSW) | ms, `uint16` |
| 57 | R | `SlaveId` | `1..247` |

Ricostruzione 32 bit:
- `cicli = HR54 << 16 | HR53`
- `tempoTotale = HR56 << 16 | HR55`

## Holding Register Write (FC06/FC16)

| HR | Tipo | Significato | Range/Comando | Persistenza EEPROM |
|---|---|---|---|---|
| 20 | W | Target posizione tapparella | `0..100` | No |
| 21 | W | `tempoTotale` corsa completa (ms) | `5000..65535` | Si |
| 22 | W | Reset cicli totali | scrivere `1` | Si |
| 23 | W | `SlaveId` Modbus | `1..247` | Si |

## EEPROM Layout

| Addr | Size | Campo |
|---|---|---|
| 0 | 1 byte | Posizione memo (`0..100`) |
| 1 | 2 byte | Magic (`0xA55A`) |
| 3 | 4 byte | Cicli totali (`uint32`) |
| 7 | 4 byte | `tempoTotale` (`unsigned long`) |
| 11 | 1 byte | `SlaveId` |

## Regole firmware rilevanti

| Regola | Comportamento |
|---|---|
| Conteggio ciclo | Incrementa solo quando posizione finale e `0%` o `100%` |
| Salvataggio cicli | Flush EEPROM ogni `CYCLES_SAVE_EVERY` (attuale `10`) |
| Salvataggio posizione | A ogni stop/fine movimento |
| Cambio Slave ID | Applicato runtime con `setUnitAddress()` + salvato EEPROM |

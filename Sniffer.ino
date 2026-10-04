#include <Arduino.h>
#include <SPI.h>
#include <ESP8266WiFi.h>

/*
 * ============================================================
 * ESP8266 + CC1101
 * METZLER 24-BIT PWM RX + TX
 * ============================================================
 *
 * Verifiziertes Protokoll des getesteten Metzler-Tasters:
 *
 * Code:
 *   0x0D8C78
 *
 * 24 Bit, MSB zuerst
 *
 * Basistakt:
 *   T ~= 303 us
 *
 * Bit 0:
 *   HIGH 1T
 *   LOW  3T
 *
 * Bit 1:
 *   HIGH 3T
 *   LOW  1T
 *
 * Sync:
 *   HIGH 1T
 *   LOW 31T
 *
 * Gesamtdauer pro Frame:
 *   128T ~= 38.8 ms
 *
 *
 * Verdrahtung:
 *
 * ESP8266           CC1101
 * --------------------------------
 * 3V3           ->  VCC
 * GND           ->  GND
 * D5 / GPIO14   ->  SCK
 * D6 / GPIO12   ->  MISO
 * D7 / GPIO13   ->  MOSI
 * D0 / GPIO16   ->  CSN
 * D1 / GPIO5    ->  GDO0   TX DATA
 * D2 / GPIO4    ->  GDO2   RX DATA
 *
 *
 * Serial Monitor:
 *   115200 Baud
 *
 *
 * Befehle:
 *
 *   h
 *       Hilfe
 *
 *   s
 *       Status anzeigen
 *
 *   t
 *       Metzler-Code 0D8C78 senden
 *
 *   t 0D8C78
 *       angegebenen 24-Bit-Code senden
 *
 *   t 0D8C78 52
 *       angegebenen Code 52x senden
 *
 *   l
 *       zuletzt bestätigten RX-Code senden
 *
 *   i
 *       TX-Polarität umschalten
 *
 *   g
 *       RX-Verstärkung zwischen NORMAL und NAHFELD umschalten
 *
 *   v
 *       Anzeige fremder gültiger 24-Bit-Codes umschalten
 *
 * ============================================================
 */


// ============================================================
// PINNING
// ============================================================

const uint8_t PIN_CS   = 16;  // D0
const uint8_t PIN_MISO = 12;  // D6

const uint8_t PIN_GDO0 = 5;   // D1 -> TX data
const uint8_t PIN_GDO2 = 4;   // D2 -> RX data


// ============================================================
// METZLER
// ============================================================

const uint32_t METZLER_CODE = 0x0D8C78UL;


// ============================================================
// RF
// ============================================================

const uint32_t RF_FREQ_KHZ = 433950UL;

/*
 * PATABLE Wert.
 *
 * 0x60 ist für den ersten Test absichtlich
 * nicht maximale Sendeleistung.
 */
const uint8_t TX_PA_LEVEL = 0x60;


/*
 * Standardmäßig normale RX-Empfindlichkeit.
 *
 * Für Tests aus wenigen Zentimetern Abstand:
 * mit "g" auf NAHFELD umschalten.
 */
bool nearFieldMode = false;


/*
 * Falls der Originalgong auf TX nicht reagiert:
 *
 *   i
 *   t
 *
 * testen.
 */
bool txInvert = false;


/*
 * Ein Frame dauert ungefähr 38.8 ms.
 *
 * 52 Frames:
 *   ungefähr 2.0 Sekunden.
 *
 * Das entspricht gut deinem beobachteten
 * Verhalten beim längeren Tastendruck.
 */
const uint16_t DEFAULT_TX_REPEATS = 52;


// ============================================================
// TX TIMINGS
// ============================================================

const uint32_t TX_T_US = 303;

const uint32_t TX_SHORT_US =
  TX_T_US;

const uint32_t TX_LONG_US =
  TX_T_US * 3UL;

const uint32_t TX_SYNC_LOW_US =
  TX_T_US * 31UL;


// ============================================================
// RX DECODER
// ============================================================

/*
 * Die gemessenen Pulse sind nicht exakt symmetrisch:
 *
 * kurzer HIGH:
 *   etwa 300...380 us
 *
 * kurzer LOW:
 *   etwa 230...310 us
 *
 * langer HIGH:
 *   etwa 900...990 us
 *
 * langer LOW:
 *   etwa 830...920 us
 *
 * Deshalb klassifizieren wir Bits hauptsächlich
 * über die Gesamtdauer und das Tastverhältnis.
 */

const uint32_t SYNC_HIGH_MIN_US = 180;
const uint32_t SYNC_HIGH_MAX_US = 520;

const uint32_t SYNC_LOW_MIN_US  = 7000;
const uint32_t SYNC_LOW_MAX_US  = 11500;


/*
 * Ein Datenbit dauert nominell 4T:
 *
 * ca. 1212 us.
 */
const uint32_t BIT_TOTAL_MIN_US = 900;
const uint32_t BIT_TOTAL_MAX_US = 1500;


/*
 * Ein echter Puls ist mindestens rund 200 us.
 *
 * Die früher beobachteten 26/52/78-us-Spikes
 * dürfen daher vorsichtig zusammengeführt werden.
 */
const uint32_t GLITCH_MAX_US = 120;


/*
 * Große Ruhephase.
 */
const uint32_t IDLE_GAP_US = 20000;


/*
 * Mindestens drei identische Frames.
 */
const uint8_t CONFIRM_FRAMES = 3;


/*
 * Normaler Frameabstand:
 * ca. 39 ms.
 *
 * Selbst wenn einige Frames verloren gehen,
 * bleiben wir mit 180 ms noch im selben Burst.
 */
const uint32_t SAME_BURST_MAX_GAP_MS = 180;


/*
 * Nach dieser Pause gilt die Betätigung als beendet.
 */
const uint32_t BURST_END_MS = 300;


bool showOtherCodes = false;


// ============================================================
// CC1101 REGISTER
// ============================================================

#define CC_IOCFG2      0x00
#define CC_IOCFG0      0x02

#define CC_PKTCTRL1    0x07
#define CC_PKTCTRL0    0x08

#define CC_FSCTRL1     0x0B
#define CC_FSCTRL0     0x0C

#define CC_FREQ2       0x0D
#define CC_FREQ1       0x0E
#define CC_FREQ0       0x0F

#define CC_MDMCFG4     0x10
#define CC_MDMCFG3     0x11
#define CC_MDMCFG2     0x12
#define CC_MDMCFG1     0x13
#define CC_MDMCFG0     0x14

#define CC_MCSM1       0x17
#define CC_MCSM0       0x18

#define CC_FOCCFG      0x19
#define CC_BSCFG       0x1A

#define CC_AGCCTRL2    0x1B
#define CC_AGCCTRL1    0x1C
#define CC_AGCCTRL0    0x1D

#define CC_FREND1      0x21
#define CC_FREND0      0x22

#define CC_FSCAL3      0x23
#define CC_FSCAL2      0x24
#define CC_FSCAL1      0x25
#define CC_FSCAL0      0x26

#define CC_TEST2       0x2C
#define CC_TEST1       0x2D
#define CC_TEST0       0x2E

#define CC_PARTNUM     0x30
#define CC_VERSION     0x31
#define CC_RSSI        0x34
#define CC_MARCSTATE   0x35

#define CC_PATABLE     0x3E


// Command strobes

#define CC_SRES        0x30
#define CC_SCAL        0x33
#define CC_SRX         0x34
#define CC_STX         0x35
#define CC_SIDLE       0x36


// MARCSTATE

#define MARC_IDLE      0x01
#define MARC_RX        0x0D
#define MARC_TX        0x13


// ============================================================
// AGC
// ============================================================

/*
 * Normal:
 * volle Empfindlichkeit.
 */
const uint8_t AGCCTRL2_NORMAL =
  0x06;


/*
 * Nahfeld:
 * maximale LNA-Verstärkung stark begrenzt.
 *
 * Dies ist die Einstellung, mit der der
 * Metzler aus wenigen Zentimetern extrem
 * sauber isoliert wurde.
 */
const uint8_t AGCCTRL2_NEAR =
  0x3E;


// ============================================================
// RAW RINGBUFFER
// ============================================================

const uint16_t RAW_RING_SIZE = 2048;
const uint16_t RAW_RING_MASK = RAW_RING_SIZE - 1;

volatile int32_t rawRing[RAW_RING_SIZE];

volatile uint16_t rawHead = 0;
volatile uint16_t rawTail = 0;

volatile bool rawOverflow = false;

volatile uint32_t lastEdgeUs = 0;

volatile bool currentGdoLevel = false;


// ============================================================
// FILTERBUFFER
// ============================================================

const uint16_t FILTER_SIZE = 192;

int32_t filtered[FILTER_SIZE];

uint16_t filteredCount = 0;


// ============================================================
// BURST CONFIRMATION
// ============================================================

bool candidateActive = false;

uint32_t candidateCode = 0;

uint16_t candidateFrameCount = 0;

uint32_t candidateLastMs = 0;

bool candidateAnnounced = false;


uint32_t lastConfirmedCode = 0;

bool haveLastConfirmedCode = false;


// ============================================================
// SERIAL
// ============================================================

char commandBuffer[64];

uint8_t commandLength = 0;


// ============================================================
// HILFSFUNKTIONEN
// ============================================================

uint32_t absPulse(
  int32_t value
) {

  if (value < 0) {

    return
      (uint32_t)(-value);
  }

  return
    (uint32_t)value;
}


bool sameSign(
  int32_t a,
  int32_t b
) {

  return
    (
      a >= 0 &&
      b >= 0
    )
    ||
    (
      a < 0 &&
      b < 0
    );
}


bool inRange(
  uint32_t value,
  uint32_t minimum,
  uint32_t maximum
) {

  return
    value >= minimum &&
    value <= maximum;
}


// ============================================================
// HEX AUSGABE
// ============================================================

void printCode24(
  uint32_t code
) {

  code &=
    0xFFFFFFUL;


  for (
    int8_t nibble = 5;
    nibble >= 0;
    nibble--
  ) {

    uint8_t value =
      (
        code >>
        (nibble * 4)
      ) & 0x0F;


    if (value < 10) {

      Serial.print(
        (char)(
          '0' + value
        )
      );

    } else {

      Serial.print(
        (char)(
          'A' +
          value -
          10
        )
      );
    }
  }
}


// ============================================================
// SPI
// ============================================================

bool waitReady(
  uint32_t timeoutUs = 10000
) {

  uint32_t start =
    micros();


  while (
    digitalRead(PIN_MISO) ==
    HIGH
  ) {

    if (
      (uint32_t)(
        micros() - start
      ) >
      timeoutUs
    ) {

      return false;
    }
  }


  return true;
}


bool selectCC() {

  digitalWrite(
    PIN_CS,
    LOW
  );


  if (!waitReady()) {

    digitalWrite(
      PIN_CS,
      HIGH
    );

    return false;
  }


  return true;
}


void deselectCC() {

  digitalWrite(
    PIN_CS,
    HIGH
  );
}


uint8_t strobe(
  uint8_t command
) {

  if (!selectCC()) {

    return 0xFF;
  }


  uint8_t result =
    SPI.transfer(
      command
    );


  deselectCC();


  return result;
}


void writeReg(
  uint8_t address,
  uint8_t value
) {

  if (!selectCC()) {

    return;
  }


  SPI.transfer(
    address
  );

  SPI.transfer(
    value
  );


  deselectCC();
}


uint8_t readReg(
  uint8_t address
) {

  if (!selectCC()) {

    return 0xFF;
  }


  SPI.transfer(
    address | 0x80
  );


  uint8_t result =
    SPI.transfer(0);


  deselectCC();


  return result;
}


uint8_t readStatus(
  uint8_t address
) {

  if (!selectCC()) {

    return 0xFF;
  }


  SPI.transfer(
    address | 0xC0
  );


  uint8_t result =
    SPI.transfer(0);


  deselectCC();


  return result;
}


// ============================================================
// CC1101 RESET
// ============================================================

bool resetCC() {

  digitalWrite(
    PIN_CS,
    HIGH
  );

  delayMicroseconds(5);


  digitalWrite(
    PIN_CS,
    LOW
  );

  delayMicroseconds(10);


  digitalWrite(
    PIN_CS,
    HIGH
  );

  delayMicroseconds(50);


  digitalWrite(
    PIN_CS,
    LOW
  );


  if (!waitReady()) {

    digitalWrite(
      PIN_CS,
      HIGH
    );

    return false;
  }


  SPI.transfer(
    CC_SRES
  );


  if (!waitReady()) {

    digitalWrite(
      PIN_CS,
      HIGH
    );

    return false;
  }


  digitalWrite(
    PIN_CS,
    HIGH
  );


  delay(5);


  return true;
}


// ============================================================
// FREQUENZ
// ============================================================

void setFrequencyKHz(
  uint32_t kHz
) {

  uint64_t hz =
    (uint64_t)kHz *
    1000ULL;


  uint32_t reg =
    (uint32_t)(
      (
        hz *
        65536ULL
      ) /
      26000000ULL
    );


  writeReg(
    CC_FREQ2,
    (reg >> 16) & 0xFF
  );


  writeReg(
    CC_FREQ1,
    (reg >> 8) & 0xFF
  );


  writeReg(
    CC_FREQ0,
    reg & 0xFF
  );
}


// ============================================================
// PATABLE
// ============================================================

void writePATable() {

  if (!selectCC()) {

    return;
  }


  /*
   * Burst write PATABLE.
   *
   * PATABLE[0] -> OOK 0
   * PATABLE[1] -> OOK 1
   */
  SPI.transfer(
    CC_PATABLE | 0x40
  );


  SPI.transfer(
    0x00
  );


  SPI.transfer(
    TX_PA_LEVEL
  );


  deselectCC();
}


// ============================================================
// RSSI
// ============================================================

int16_t readRSSI() {

  int16_t raw =
    readStatus(
      CC_RSSI
    );


  if (
    raw >= 128
  ) {

    raw -= 256;
  }


  return
    raw / 2 - 74;
}


// ============================================================
// RADIO CONFIG
// ============================================================

void configureRadio() {

  strobe(
    CC_SIDLE
  );


  /*
   * GDO2:
   * asynchronous serial RX data.
   */
  writeReg(
    CC_IOCFG2,
    0x0D
  );


  /*
   * GDO0:
   * im RX unbenutzt.
   *
   * Im asynchronen TX-Modus wird GDO0
   * automatisch als TX-Dateneingang benutzt.
   */
  writeReg(
    CC_IOCFG0,
    0x2E
  );


  /*
   * Asynchronous serial mode,
   * infinite length.
   */
  writeReg(
    CC_PKTCTRL0,
    0x32
  );


  writeReg(
    CC_PKTCTRL1,
    0x04
  );


  writeReg(
    CC_FSCTRL1,
    0x06
  );


  writeReg(
    CC_FSCTRL0,
    0x00
  );


  setFrequencyKHz(
    RF_FREQ_KHZ
  );


  /*
   * ca. 203 kHz RX-Bandbreite
   * ca. 4.8 kBaud programmierte Datenrate.
   */
  writeReg(
    CC_MDMCFG4,
    0x87
  );


  writeReg(
    CC_MDMCFG3,
    0x83
  );


  /*
   * ASK/OOK, kein Sync-Handling
   * durch den CC1101.
   */
  writeReg(
    CC_MDMCFG2,
    0x30
  );


  writeReg(
    CC_MDMCFG1,
    0x02
  );


  writeReg(
    CC_MDMCFG0,
    0x00
  );


  /*
   * Kein CCA vor TX.
   */
  writeReg(
    CC_MCSM1,
    0x00
  );


  writeReg(
    CC_MCSM0,
    0x18
  );


  writeReg(
    CC_FOCCFG,
    0x16
  );


  writeReg(
    CC_BSCFG,
    0x6C
  );


  writeReg(
    CC_AGCCTRL2,
    nearFieldMode
      ? AGCCTRL2_NEAR
      : AGCCTRL2_NORMAL
  );


  writeReg(
    CC_AGCCTRL1,
    0x00
  );


  writeReg(
    CC_AGCCTRL0,
    0x91
  );


  writeReg(
    CC_FREND1,
    0x56
  );


  /*
   * PA_POWER = 1.
   *
   * ASK/OOK benutzt damit
   * PATABLE[0] und PATABLE[1].
   */
  writeReg(
    CC_FREND0,
    0x11
  );


  writeReg(
    CC_FSCAL3,
    0xE9
  );


  writeReg(
    CC_FSCAL2,
    0x2A
  );


  writeReg(
    CC_FSCAL1,
    0x00
  );


  writeReg(
    CC_FSCAL0,
    0x1F
  );


  writeReg(
    CC_TEST2,
    0x81
  );


  writeReg(
    CC_TEST1,
    0x35
  );


  writeReg(
    CC_TEST0,
    0x09
  );


  writePATable();


  strobe(
    CC_SCAL
  );


  delay(5);


  strobe(
    CC_SRX
  );


  delay(5);
}


// ============================================================
// RADIO STATE
// ============================================================

bool waitForState(
  uint8_t wanted,
  uint32_t timeoutMs
) {

  uint32_t start =
    millis();


  while (
    millis() - start <
    timeoutMs
  ) {

    uint8_t state =
      readStatus(
        CC_MARCSTATE
      ) & 0x1F;


    if (
      state ==
      wanted
    ) {

      return true;
    }


    delayMicroseconds(
      100
    );
  }


  return false;
}


// ============================================================
// ISR
// ============================================================

IRAM_ATTR void gdo2ISR() {

  uint32_t now =
    micros();


  uint32_t duration =
    now - lastEdgeUs;


  lastEdgeUs =
    now;


  /*
   * Pegel, der gerade beendet wurde.
   */
  bool endedHigh =
    currentGdoLevel;


  /*
   * CHANGE:
   * logischer Zustand wird invertiert.
   */
  currentGdoLevel =
    !currentGdoLevel;


  int32_t pulse =
    endedHigh
      ? (int32_t)duration
      : -(int32_t)duration;


  uint16_t next =
    (
      rawHead + 1
    ) &
    RAW_RING_MASK;


  if (
    next ==
    rawTail
  ) {

    rawOverflow =
      true;

    return;
  }


  rawRing[
    rawHead
  ] = pulse;


  rawHead =
    next;
}


// ============================================================
// RAW RING
// ============================================================

bool popRawPulse(
  int32_t &pulse
) {

  noInterrupts();


  if (
    rawTail ==
    rawHead
  ) {

    interrupts();

    return false;
  }


  pulse =
    rawRing[
      rawTail
    ];


  rawTail =
    (
      rawTail + 1
    ) &
    RAW_RING_MASK;


  interrupts();


  return true;
}


void clearRawQueue() {

  noInterrupts();


  rawTail =
    rawHead;


  rawOverflow =
    false;


  interrupts();


  filteredCount =
    0;
}


// ============================================================
// RX INTERRUPT
// ============================================================

void enableReceiverInterrupt() {

  clearRawQueue();


  currentGdoLevel =
    digitalRead(
      PIN_GDO2
    );


  lastEdgeUs =
    micros();


  attachInterrupt(
    digitalPinToInterrupt(
      PIN_GDO2
    ),
    gdo2ISR,
    CHANGE
  );
}


void disableReceiverInterrupt() {

  detachInterrupt(
    digitalPinToInterrupt(
      PIN_GDO2
    )
  );


  clearRawQueue();
}


// ============================================================
// FILTER BUFFER
// ============================================================

void consumeFiltered(
  uint16_t count
) {

  if (
    count >= filteredCount
  ) {

    filteredCount = 0;

    return;
  }


  memmove(
    filtered,
    filtered + count,
    (
      filteredCount -
      count
    ) *
    sizeof(
      filtered[0]
    )
  );


  filteredCount -=
    count;
}


// ============================================================
// BURST RESET
// ============================================================

void resetCandidate() {

  candidateActive =
    false;

  candidateCode =
    0;

  candidateFrameCount =
    0;

  candidateLastMs =
    0;

  candidateAnnounced =
    false;
}


// ============================================================
// FRAME EMPFANGEN
// ============================================================

void validFrame(
  uint32_t code
) {

  code &=
    0xFFFFFFUL;


  uint32_t now =
    millis();


  /*
   * Neuer Burst oder anderer Code.
   */
  if (
    !candidateActive
    ||
    code != candidateCode
    ||
    (
      now -
      candidateLastMs
    ) >
    SAME_BURST_MAX_GAP_MS
  ) {

    candidateActive =
      true;


    candidateCode =
      code;


    candidateFrameCount =
      1;


    candidateAnnounced =
      false;

  } else {

    if (
      candidateFrameCount <
      65535
    ) {

      candidateFrameCount++;
    }
  }


  candidateLastMs =
    now;


  /*
   * Optional jedes Einzel-Frame fremder
   * Sender anzeigen.
   */
  if (
    showOtherCodes &&
    code != METZLER_CODE
  ) {

    Serial.print(
      F("RX FRAME: ")
    );

    printCode24(
      code
    );

    Serial.println();
  }


  /*
   * Erst nach mehreren identischen Frames
   * eine Meldung ausgeben.
   */
  if (
    !candidateAnnounced &&
    candidateFrameCount >=
    CONFIRM_FRAMES
  ) {

    candidateAnnounced =
      true;


    lastConfirmedCode =
      code;


    haveLastConfirmedCode =
      true;


    if (
      code ==
      METZLER_CODE
    ) {

      Serial.println();

      Serial.print(
        F("METZLER RX: ")
      );

      printCode24(
        code
      );

      Serial.print(
        F("  Frames=")
      );

      Serial.println(
        candidateFrameCount
      );

      Serial.println(
        F("*** KLINGEL ERKANNT ***")
      );

      Serial.println();

    }

    else if (
      showOtherCodes
    ) {

      Serial.print(
        F("RF24 CONFIRMED: ")
      );

      printCode24(
        code
      );

      Serial.print(
        F("  Frames=")
      );

      Serial.println(
        candidateFrameCount
      );
    }
  }
}


// ============================================================
// EIN BIT DECODIEREN
// ============================================================

bool decodeBitPair(
  int32_t highPulse,
  int32_t lowPulse,
  uint8_t &bit
) {

  /*
   * Erwartete Polarität:
   *
   * +HIGH
   * -LOW
   */
  if (
    highPulse <= 0 ||
    lowPulse >= 0
  ) {

    return false;
  }


  uint32_t highUs =
    (uint32_t)highPulse;


  uint32_t lowUs =
    (uint32_t)(
      -lowPulse
    );


  uint32_t total =
    highUs +
    lowUs;


  if (
    !inRange(
      total,
      BIT_TOTAL_MIN_US,
      BIT_TOTAL_MAX_US
    )
  ) {

    return false;
  }


  /*
   * Bit 0:
   *
   * HIGH ~= 25 %
   *
   * Bit 1:
   *
   * HIGH ~= 75 %
   *
   * Ein großer neutraler Bereich bleibt
   * absichtlich ungültig.
   */
  uint32_t highPercent =
    (
      highUs *
      100UL
    ) /
    total;


  if (
    highPercent <= 45
  ) {

    bit = 0;

    return true;
  }


  if (
    highPercent >= 55
  ) {

    bit = 1;

    return true;
  }


  return false;
}


// ============================================================
// 24-BIT FRAME SUCHEN
// ============================================================

bool tryDecodeFrame() {

  /*
   * Sync:
   *   2 Pulse
   *
   * Daten:
   *   24 * 2 Pulse
   *
   * Gesamt:
   *   50 Pulse
   */
  const uint16_t FRAME_PULSES =
    50;


  if (
    filteredCount <
    FRAME_PULSES
  ) {

    return false;
  }


  for (
    uint16_t start = 0;
    start + FRAME_PULSES
      <= filteredCount;
    start++
  ) {

    int32_t syncHigh =
      filtered[
        start
      ];


    int32_t syncLow =
      filtered[
        start + 1
      ];


    if (
      syncHigh <= 0 ||
      syncLow >= 0
    ) {

      continue;
    }


    if (
      !inRange(
        (uint32_t)syncHigh,
        SYNC_HIGH_MIN_US,
        SYNC_HIGH_MAX_US
      )
    ) {

      continue;
    }


    if (
      !inRange(
        (uint32_t)(
          -syncLow
        ),
        SYNC_LOW_MIN_US,
        SYNC_LOW_MAX_US
      )
    ) {

      continue;
    }


    uint32_t code = 0;

    bool valid = true;


    for (
      uint8_t bitIndex = 0;
      bitIndex < 24;
      bitIndex++
    ) {

      uint16_t pairIndex =
        start +
        2 +
        bitIndex * 2;


      uint8_t bit;


      if (
        !decodeBitPair(
          filtered[
            pairIndex
          ],
          filtered[
            pairIndex + 1
          ],
          bit
        )
      ) {

        valid = false;

        break;
      }


      code <<= 1;

      code |=
        bit;
    }


    if (!valid) {

      continue;
    }


    validFrame(
      code
    );


    /*
     * Erkanntes Frame entfernen.
     *
     * Das nächste Frame beginnt danach
     * direkt wieder mit dem Sync-HIGH.
     */
    consumeFiltered(
      start +
      FRAME_PULSES
    );


    return true;
  }


  return false;
}


// ============================================================
// FILTERBUFFER BESCHRÄNKEN
// ============================================================

void trimFiltered() {

  const uint16_t KEEP =
    100;


  if (
    filteredCount <=
    KEEP
  ) {

    return;
  }


  consumeFiltered(
    filteredCount -
    KEEP
  );
}


// ============================================================
// RAW -> GLITCHFILTER -> DECODER
// ============================================================

void processRawPulse(
  int32_t pulse
) {

  uint32_t duration =
    absPulse(
      pulse
    );


  /*
   * Lange Ruhephase.
   *
   * Sie gehört nicht mehr zu einem Frame.
   */
  if (
    duration >
    IDLE_GAP_US
  ) {

    filteredCount =
      0;

    return;
  }


  if (
    filteredCount >=
    FILTER_SIZE
  ) {

    consumeFiltered(
      64
    );
  }


  filtered[
    filteredCount++
  ] = pulse;


  /*
   * Sicherer Glitch-Filter:
   *
   * Beispiel:
   *
   * +900
   * -26
   * +300
   *
   * wird zu einem einzigen positiven Puls.
   *
   * GLITCH_MAX_US = 120 ist bewusst deutlich
   * kleiner als der kleinste echte Metzler-Puls.
   */
  bool merged =
    true;


  while (
    merged &&
    filteredCount >= 3
  ) {

    merged =
      false;


    uint16_t aIndex =
      filteredCount - 3;

    uint16_t bIndex =
      filteredCount - 2;

    uint16_t cIndex =
      filteredCount - 1;


    int32_t a =
      filtered[aIndex];

    int32_t b =
      filtered[bIndex];

    int32_t c =
      filtered[cIndex];


    if (
      absPulse(b) <=
      GLITCH_MAX_US
      &&
      sameSign(
        a,
        c
      )
    ) {

      uint32_t durationMerged =
        absPulse(a)
        +
        absPulse(b)
        +
        absPulse(c);


      filtered[aIndex] =
        a >= 0
          ? (int32_t)durationMerged
          : -(int32_t)durationMerged;


      filteredCount -=
        2;


      merged =
        true;
    }
  }


  while (
    tryDecodeFrame()
  ) {

    // ggf. mehrere Frames im Buffer
  }


  trimFiltered();
}


// ============================================================
// RX SERVICE
// ============================================================

void serviceReceiver() {

  if (
    rawOverflow
  ) {

    Serial.println(
      F("WARNUNG: RX Ringbuffer Overflow")
    );


    clearRawQueue();


    return;
  }


  int32_t pulse;


  while (
    popRawPulse(
      pulse
    )
  ) {

    processRawPulse(
      pulse
    );
  }


  /*
   * Ende einer Betätigung.
   */
  if (
    candidateActive &&
    (
      millis() -
      candidateLastMs
    ) >
    BURST_END_MS
  ) {

    resetCandidate();
  }
}


// ============================================================
// TX LEVEL
// ============================================================

void setTxLevel(
  bool logicalHigh
) {

  bool output =
    txInvert
      ? !logicalHigh
      : logicalHigh;


  digitalWrite(
    PIN_GDO0,
    output
      ? HIGH
      : LOW
  );
}


// ============================================================
// TX PHASE
// ============================================================

void txPhase(
  bool high,
  uint32_t durationUs
) {

  setTxLevel(
    high
  );


  delayMicroseconds(
    durationUs
  );
}


// ============================================================
// EIN 24-BIT FRAME SENDEN
// ============================================================

void sendFrame24(
  uint32_t code
) {

  code &=
    0xFFFFFFUL;


  /*
   * SYNC
   *
   * 1T HIGH
   * 31T LOW
   */
  txPhase(
    true,
    TX_SHORT_US
  );


  txPhase(
    false,
    TX_SYNC_LOW_US
  );


  /*
   * 24 Bit, MSB zuerst.
   */
  for (
    int8_t bit = 23;
    bit >= 0;
    bit--
  ) {

    bool one =
      (
        code >>
        bit
      ) & 1UL;


    if (one) {

      /*
       * 1:
       * 3T HIGH
       * 1T LOW
       */
      txPhase(
        true,
        TX_LONG_US
      );


      txPhase(
        false,
        TX_SHORT_US
      );

    } else {

      /*
       * 0:
       * 1T HIGH
       * 3T LOW
       */
      txPhase(
        true,
        TX_SHORT_US
      );


      txPhase(
        false,
        TX_LONG_US
      );
    }
  }
}


// ============================================================
// SENDEN
// ============================================================

bool sendCode(
  uint32_t code,
  uint16_t repeats
) {

  code &=
    0xFFFFFFUL;


  if (
    repeats < 1
  ) {

    repeats = 1;
  }


  if (
    repeats > 200
  ) {

    repeats = 200;
  }


  Serial.println();

  Serial.print(
    F("METZLER TX: ")
  );


  printCode24(
    code
  );


  Serial.print(
    F("  repeats=")
  );


  Serial.print(
    repeats
  );


  Serial.print(
    F("  Dauer~")
  );


  uint32_t durationMs =
    (
      (uint32_t)repeats *
      128UL *
      TX_T_US
    ) /
    1000UL;


  Serial.print(
    durationMs
  );


  Serial.print(
    F(" ms  invert=")
  );


  Serial.println(
    txInvert
      ? F("JA")
      : F("NEIN")
  );


  disableReceiverInterrupt();


  resetCandidate();


  /*
   * GDO0 vor TX physikalisch LOW.
   */
  digitalWrite(
    PIN_GDO0,
    LOW
  );


  strobe(
    CC_SIDLE
  );


  delayMicroseconds(
    300
  );


  strobe(
    CC_STX
  );


  if (
    !waitForState(
      MARC_TX,
      50
    )
  ) {

    Serial.println(
      F("FEHLER: CC1101 erreicht TX nicht.")
    );


    strobe(
      CC_SIDLE
    );


    strobe(
      CC_SRX
    );


    waitForState(
      MARC_RX,
      50
    );


    enableReceiverInterrupt();


    return false;
  }


  /*
   * Synthesizer kurz stabilisieren lassen.
   */
  delayMicroseconds(
    300
  );


  for (
    uint16_t i = 0;
    i < repeats;
    i++
  ) {

    sendFrame24(
      code
    );


    /*
     * Watchdog bedienen, ohne yield()
     * und damit ohne größere Timing-Lücke
     * zwischen den Frames.
     */
    ESP.wdtFeed();
  }


  /*
   * Vor Verlassen des TX-Zustands
   * den physischen Eingang LOW setzen.
   */
  digitalWrite(
    PIN_GDO0,
    LOW
  );


  delayMicroseconds(
    200
  );


  strobe(
    CC_SIDLE
  );


  delayMicroseconds(
    200
  );


  strobe(
    CC_SRX
  );


  if (
    !waitForState(
      MARC_RX,
      50
    )
  ) {

    Serial.println(
      F("WARNUNG: RX-State nicht erreicht.")
    );
  }


  delay(
    3
  );


  enableReceiverInterrupt();


  Serial.println(
    F("TX fertig, wieder RX.")
  );


  Serial.println();


  return true;
}


// ============================================================
// RX GAIN UMSCHALTEN
// ============================================================

void applyRxGain() {

  disableReceiverInterrupt();


  strobe(
    CC_SIDLE
  );


  writeReg(
    CC_AGCCTRL2,
    nearFieldMode
      ? AGCCTRL2_NEAR
      : AGCCTRL2_NORMAL
  );


  strobe(
    CC_SRX
  );


  delay(
    3
  );


  enableReceiverInterrupt();


  Serial.print(
    F("RX Gain: ")
  );


  Serial.println(
    nearFieldMode
      ? F("NAHFELD")
      : F("NORMAL")
  );
}


// ============================================================
// HEX PARSER
// ============================================================

int8_t hexValue(
  char c
) {

  if (
    c >= '0' &&
    c <= '9'
  ) {

    return
      c - '0';
  }


  if (
    c >= 'A' &&
    c <= 'F'
  ) {

    return
      c - 'A' + 10;
  }


  if (
    c >= 'a' &&
    c <= 'f'
  ) {

    return
      c - 'a' + 10;
  }


  return -1;
}


bool parseCode24(
  const char *text,
  uint32_t &code
) {

  if (
    text == nullptr
  ) {

    return false;
  }


  if (
    text[0] == '0' &&
    (
      text[1] == 'x' ||
      text[1] == 'X'
    )
  ) {

    text += 2;
  }


  uint32_t value = 0;

  uint8_t digits = 0;


  while (*text) {

    int8_t h =
      hexValue(
        *text
      );


    if (
      h < 0
    ) {

      return false;
    }


    if (
      digits >= 6
    ) {

      return false;
    }


    value <<= 4;

    value |=
      (uint8_t)h;


    digits++;

    text++;
  }


  if (
    digits != 6
  ) {

    return false;
  }


  code =
    value;


  return true;
}


// ============================================================
// STATUS
// ============================================================

void printStatus() {

  uint8_t marc =
    readStatus(
      CC_MARCSTATE
    ) & 0x1F;


  Serial.println();

  Serial.print(
    F("MARCSTATE : 0x")
  );


  if (
    marc < 0x10
  ) {

    Serial.print('0');
  }


  Serial.println(
    marc,
    HEX
  );


  Serial.print(
    F("RSSI      : ")
  );

  Serial.print(
    readRSSI()
  );

  Serial.println(
    F(" dBm")
  );


  Serial.print(
    F("RX Gain   : ")
  );

  Serial.println(
    nearFieldMode
      ? F("NAHFELD")
      : F("NORMAL")
  );


  Serial.print(
    F("TX invert : ")
  );

  Serial.println(
    txInvert
      ? F("JA")
      : F("NEIN")
  );


  Serial.print(
    F("Metzler   : ")
  );

  printCode24(
    METZLER_CODE
  );

  Serial.println();


  Serial.print(
    F("Letzter RX: ")
  );


  if (
    haveLastConfirmedCode
  ) {

    printCode24(
      lastConfirmedCode
    );

    Serial.println();

  } else {

    Serial.println(
      F("-")
    );
  }


  Serial.print(
    F("Andere Codes anzeigen: ")
  );

  Serial.println(
    showOtherCodes
      ? F("JA")
      : F("NEIN")
  );


  Serial.println();
}


// ============================================================
// HELP
// ============================================================

void printHelp() {

  Serial.println();

  Serial.println(
    F("Befehle:")
  );

  Serial.println(
    F("  t")
  );

  Serial.println(
    F("      Metzler 0D8C78 ca. 2 Sekunden senden")
  );

  Serial.println(
    F("  t 0D8C78")
  );

  Serial.println(
    F("      beliebigen 24-Bit-Code senden")
  );

  Serial.println(
    F("  t 0D8C78 52")
  );

  Serial.println(
    F("      Code mit eigener Wiederholungszahl senden")
  );

  Serial.println(
    F("  l")
  );

  Serial.println(
    F("      letzten bestaetigten RX-Code senden")
  );

  Serial.println(
    F("  i")
  );

  Serial.println(
    F("      TX-Polaritaet umschalten")
  );

  Serial.println(
    F("  g")
  );

  Serial.println(
    F("      RX Gain NORMAL / NAHFELD")
  );

  Serial.println(
    F("  v")
  );

  Serial.println(
    F("      fremde gueltige 24-Bit-Codes anzeigen")
  );

  Serial.println(
    F("  s")
  );

  Serial.println(
    F("      Status")
  );

  Serial.println(
    F("  h")
  );

  Serial.println(
    F("      Hilfe")
  );

  Serial.println();
}


// ============================================================
// COMMAND
// ============================================================

void executeCommand(
  char *line
) {

  char *command =
    strtok(
      line,
      " "
    );


  if (
    command == nullptr
  ) {

    return;
  }


  if (
    command[0] == 'h' ||
    command[0] == 'H'
  ) {

    printHelp();

    return;
  }


  if (
    command[0] == 's' ||
    command[0] == 'S'
  ) {

    printStatus();

    return;
  }


  if (
    command[0] == 'i' ||
    command[0] == 'I'
  ) {

    txInvert =
      !txInvert;


    Serial.print(
      F("TX invert = ")
    );


    Serial.println(
      txInvert
        ? F("JA")
        : F("NEIN")
    );


    return;
  }


  if (
    command[0] == 'g' ||
    command[0] == 'G'
  ) {

    nearFieldMode =
      !nearFieldMode;


    applyRxGain();


    return;
  }


  if (
    command[0] == 'v' ||
    command[0] == 'V'
  ) {

    showOtherCodes =
      !showOtherCodes;


    Serial.print(
      F("Andere Codes anzeigen = ")
    );


    Serial.println(
      showOtherCodes
        ? F("JA")
        : F("NEIN")
    );


    return;
  }


  if (
    command[0] == 'l' ||
    command[0] == 'L'
  ) {

    if (
      !haveLastConfirmedCode
    ) {

      Serial.println(
        F("Noch kein bestaetigter RX-Code.")
      );

      return;
    }


    sendCode(
      lastConfirmedCode,
      DEFAULT_TX_REPEATS
    );


    return;
  }


  if (
    command[0] == 't' ||
    command[0] == 'T'
  ) {

    char *codeText =
      strtok(
        nullptr,
        " "
      );


    char *repeatText =
      strtok(
        nullptr,
        " "
      );


    uint32_t code =
      METZLER_CODE;


    uint16_t repeats =
      DEFAULT_TX_REPEATS;


    /*
     * "t" alleine:
     * bekannten Metzler-Code senden.
     */
    if (
      codeText != nullptr
    ) {

      if (
        !parseCode24(
          codeText,
          code
        )
      ) {

        Serial.println(
          F("Fehler: 6 Hex-Zeichen erwartet.")
        );

        Serial.println(
          F("Beispiel: t 0D8C78")
        );

        return;
      }
    }


    if (
      repeatText != nullptr
    ) {

      long requested =
        atol(
          repeatText
        );


      if (
        requested < 1 ||
        requested > 200
      ) {

        Serial.println(
          F("Fehler: repeats muss 1..200 sein.")
        );

        return;
      }


      repeats =
        (uint16_t)requested;
    }


    sendCode(
      code,
      repeats
    );


    return;
  }


  Serial.println(
    F("Unbekannter Befehl. h = Hilfe")
  );
}


// ============================================================
// SERIAL
// ============================================================

void serviceSerial() {

  while (
    Serial.available()
  ) {

    char c =
      Serial.read();


    if (
      c == '\r' ||
      c == '\n'
    ) {

      if (
        commandLength > 0
      ) {

        commandBuffer[
          commandLength
        ] = 0;


        executeCommand(
          commandBuffer
        );


        commandLength =
          0;
      }


      continue;
    }


    if (
      commandLength <
      sizeof(commandBuffer) - 1
    ) {

      commandBuffer[
        commandLength++
      ] = c;
    }
  }
}


// ============================================================
// REGISTERAUSGABE
// ============================================================

void printHexReg(
  const char *name,
  uint8_t value
) {

  Serial.print(
    name
  );


  Serial.print(
    F(": 0x")
  );


  if (
    value < 0x10
  ) {

    Serial.print('0');
  }


  Serial.println(
    value,
    HEX
  );
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(
    115200
  );


  delay(
    300
  );


  /*
   * WLAN für saubere RF-/Timing-Tests abschalten.
   */
  WiFi.mode(
    WIFI_OFF
  );


  WiFi.forceSleepBegin();


  delay(
    10
  );


  pinMode(
    PIN_CS,
    OUTPUT
  );


  digitalWrite(
    PIN_CS,
    HIGH
  );


  pinMode(
    PIN_MISO,
    INPUT
  );


  pinMode(
    PIN_GDO0,
    OUTPUT
  );


  digitalWrite(
    PIN_GDO0,
    LOW
  );


  pinMode(
    PIN_GDO2,
    INPUT
  );


  SPI.begin();


  SPI.beginTransaction(
    SPISettings(
      1000000,
      MSBFIRST,
      SPI_MODE0
    )
  );


  Serial.println();

  Serial.println(
    F("========================================")
  );

  Serial.println(
    F(" CC1101 METZLER 24-BIT RX + TX")
  );

  Serial.println(
    F("========================================")
  );


  if (
    !resetCC()
  ) {

    Serial.println(
      F("CC1101 RESET FEHLER")
    );


    while (true) {

      ESP.wdtFeed();
    }
  }


  configureRadio();


  printHexReg(
    "PARTNUM ",
    readStatus(
      CC_PARTNUM
    )
  );


  printHexReg(
    "VERSION ",
    readStatus(
      CC_VERSION
    )
  );


  printHexReg(
    "IOCFG2  ",
    readReg(
      CC_IOCFG2
    )
  );


  printHexReg(
    "PKTCTRL0",
    readReg(
      CC_PKTCTRL0
    )
  );


  printHexReg(
    "MDMCFG4 ",
    readReg(
      CC_MDMCFG4
    )
  );


  printHexReg(
    "MDMCFG3 ",
    readReg(
      CC_MDMCFG3
    )
  );


  printHexReg(
    "MDMCFG2 ",
    readReg(
      CC_MDMCFG2
    )
  );


  printHexReg(
    "AGCCTRL2",
    readReg(
      CC_AGCCTRL2
    )
  );


  printHexReg(
    "AGCCTRL1",
    readReg(
      CC_AGCCTRL1
    )
  );


  printHexReg(
    "AGCCTRL0",
    readReg(
      CC_AGCCTRL0
    )
  );


  printHexReg(
    "FREND0  ",
    readReg(
      CC_FREND0
    )
  );


  uint8_t marc =
    readStatus(
      CC_MARCSTATE
    ) & 0x1F;


  Serial.print(
    F("MARCSTATE: 0x")
  );


  if (
    marc < 0x10
  ) {

    Serial.print('0');
  }


  Serial.println(
    marc,
    HEX
  );


  Serial.println();

  Serial.print(
    F("Metzler Code: ")
  );


  printCode24(
    METZLER_CODE
  );


  Serial.println();


  Serial.println(
    F("RX Gain: NORMAL")
  );


  enableReceiverInterrupt();


  Serial.println();
  Serial.println(
    F("Empfaenger aktiv.")
  );

  Serial.println(
    F("Zum Senden: t")
  );


  printHelp();
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  serviceReceiver();

  serviceSerial();

  yield();
}
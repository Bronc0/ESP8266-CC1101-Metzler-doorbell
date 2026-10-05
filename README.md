# ESP8266-CC1101-Metzler-doorbell
# ESP8266 + CC1101 Metzler 24-Bit RF Receiver & Transmitter

Arduino sketch for **receiving, decoding, and transmitting a Metzler 24-bit RF signal** using an **ESP8266** and a **CC1101 433 MHz transceiver**.

The sketch communicates with the CC1101 directly over SPI and does not require an external CC1101 library. Incoming RF pulses are captured using interrupts, filtered, decoded, and validated before a received code is reported.

## Features

- Receive 24-bit PWM RF signals
- Transmit arbitrary 24-bit codes
- Support for the Metzler RF codes
- CC1101 ASK/OOK operation
- Interrupt-driven RX via `GDO2`
- Asynchronous TX via `GDO0`
- Pulse glitch filtering
- Multi-frame validation to reduce false detections
- Switchable normal / near-field RX gain
- Switchable TX polarity
- RSSI and CC1101 status output
- Optional display of other valid 24-bit codes
- Serial Monitor command interface
- No external CC1101 library required

---

## Hardware

Required components:

- ESP8266 development board
- CC1101 RF transceiver
- Suitable 433 MHz antenna
- Jumper wires

The sketch is configured for:

```text
433.950 MHz
```

> **Important:** The CC1101 is a 3.3 V device. Do not power it from 5 V.

---

## Wiring

| ESP8266 | GPIO | CC1101 | Function |
|---|---:|---|---|
| 3V3 | — | VCC | Power |
| GND | — | GND | Ground |
| D5 | GPIO14 | SCK | SPI Clock |
| D6 | GPIO12 | MISO | SPI MISO |
| D7 | GPIO13 | MOSI | SPI MOSI |
| D0 | GPIO16 | CSN | Chip Select |
| D1 | GPIO5 | GDO0 | TX Data |
| D2 | GPIO4 | GDO2 | RX Data |

```text
ESP8266                 CC1101
--------------------------------
3V3       ----------->  VCC
GND       ----------->  GND
D5/GPIO14 ----------->  SCK
D6/GPIO12 ----------->  MISO
D7/GPIO13 ----------->  MOSI
D0/GPIO16 ----------->  CSN
D1/GPIO5  ----------->  GDO0
D2/GPIO4  ----------->  GDO2
```

---

## Metzler RF Protocol

The tested Metzler transmitter uses a **24-bit PWM protocol**, transmitted **MSB first**.

Known code:

```text
0x0D8C78
```

### Base timing

```text
T ≈ 303 µs
```

### Bit `0`

```text
HIGH  1T
LOW   3T
```

Approximately:

```text
HIGH   303 µs
LOW    909 µs
```

### Bit `1`

```text
HIGH  3T
LOW   1T
```

Approximately:

```text
HIGH   909 µs
LOW    303 µs
```

### Sync pulse

```text
HIGH   1T
LOW   31T
```

Approximately:

```text
HIGH    303 µs
LOW    9393 µs
```

### Frame duration

A complete frame consists of:

```text
128T ≈ 38.8 ms
```

---

## How Receiving Works

The CC1101 outputs asynchronous RF data through `GDO2`.

The ESP8266 monitors both signal edges using an interrupt and stores the measured pulse durations in a ring buffer.

The decoder then:

1. Captures HIGH and LOW pulse durations
2. Filters short glitches
3. Searches for a valid synchronization pulse
4. Decodes the following 24 bits
5. Groups identical frames into a burst
6. Confirms a code only after multiple matching frames

By default, a received code must appear in at least:

```text
3 identical frames
```

before it is considered valid.

For the known Metzler code, the Serial Monitor may show:

```text
METZLER RX: 0D8C78  Frames=3
*** KLINGEL ERKANNT ***
```

---

## Glitch Filtering

Short interference pulses can appear in the raw RF signal.

The sketch treats pulses up to:

```text
120 µs
```

as potential glitches.

For example:

```text
+900
-26
+300
```

can be merged into a single positive pulse.

This makes the decoder more tolerant of short RF disturbances without changing the actual Metzler timing.

---

## RX Gain Modes

The sketch provides two receiver gain modes.

### NORMAL

Default mode with normal receiver sensitivity.

```text
AGCCTRL2 = 0x06
```

### NEAR FIELD

Reduced LNA gain for testing a transmitter only a few centimeters away from the receiver.

```text
AGCCTRL2 = 0x3E
```

Toggle between both modes with:

```text
g
```

This can be useful when a nearby transmitter overloads the receiver input.

---

## Transmitting

When a transmission is started, the sketch:

1. Disables the RX interrupt
2. Switches the CC1101 to TX mode
3. Generates the complete PWM waveform through `GDO0`
4. Repeats the frame multiple times
5. Switches the CC1101 back to RX mode
6. Re-enables the receiver interrupt

The default number of repetitions is:

```text
52
```

With approximately `38.8 ms` per frame, this results in a transmission duration of roughly:

```text
2 seconds
```

The number of repetitions can be configured between:

```text
1 ... 200
```

---

## TX Polarity

The transmitted data polarity can be inverted at runtime.

If the original receiver or doorbell does not react to a transmitted code, toggle TX polarity with:

```text
i
```

and try again:

```text
t
```

---

## Serial Monitor

Open the Serial Monitor at:

```text
115200 baud
```

Commands are entered as plain text followed by Enter.

### Command Overview

| Command | Description |
|---|---|
| `h` | Show help |
| `s` | Show current radio status |
| `t` | Transmit the default Metzler code `0D8C78` |
| `t 0D8C78` | Transmit the specified 24-bit code |
| `t 0D8C78 52` | Transmit the code with a custom repeat count |
| `l` | Re-transmit the last confirmed RX code |
| `i` | Toggle TX polarity |
| `g` | Toggle RX gain between NORMAL and NEAR FIELD |
| `v` | Toggle display of other valid 24-bit codes |

---

## Usage Examples

### Transmit the default Metzler code

```text
t
```

This sends:

```text
0D8C78
```

using the default repeat count.

---

### Transmit a custom 24-bit code

```text
t A1B2C3
```

The code must contain exactly six hexadecimal digits.

The optional `0x` prefix is also accepted:

```text
t 0xA1B2C3
```

---

### Use a custom repeat count

```text
t 0D8C78 20
```

This transmits `0D8C78` exactly 20 times.

Valid values are:

```text
1 ... 200
```

---

### Re-transmit the last received code

```text
l
```

The most recently confirmed 24-bit code is transmitted using the default repeat count.

---

### Display other valid RF codes

```text
v
```

This enables or disables output for other valid 24-bit codes.

This is useful when testing additional transmitters or reverse-engineering compatible devices.

---

## Status Output

Enter:

```text
s
```

to display the current radio status.

The output includes:

```text
MARCSTATE
RSSI
RX Gain
TX invert
Metzler code
Last RX code
Other-code display state
```

Example:

```text
MARCSTATE : 0x0D
RSSI      : -74 dBm
RX Gain   : NORMAL
TX invert : NEIN
Metzler   : 0D8C78
Letzter RX: 0D8C78
Andere Codes anzeigen: NEIN
```

Some status strings inside the sketch are still in German, but this does not affect operation.

---

## CC1101 Configuration

The CC1101 is configured directly through its registers.

Important settings include:

| Parameter | Value |
|---|---|
| RF frequency | `433.950 MHz` |
| Modulation | ASK/OOK |
| Data mode | Asynchronous Serial |
| RX bandwidth | approximately `203 kHz` |
| Configured data rate | approximately `4.8 kBaud` |
| SPI clock | `1 MHz` |
| SPI mode | `SPI_MODE0` |
| Bit order | `MSBFIRST` |
| TX PATABLE value | `0x60` |

The CC1101's built-in packet and sync handling is not used for decoding the Metzler signal.

Instead, the sketch processes the raw asynchronous RF waveform itself.

---

## Installation

The sketch uses:

```cpp
#include <Arduino.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
```

### Requirements

- Arduino IDE or compatible Arduino build environment
- ESP8266 Arduino Core
- ESP8266 development board
- CC1101 module

No separate CC1101 library is required.

### Upload

1. Install ESP8266 board support in your Arduino environment.
2. Connect the CC1101 according to the wiring table.
3. Open the sketch.
4. Select the correct ESP8266 board and serial port.
5. Compile and upload the sketch.
6. Open the Serial Monitor.
7. Set the baud rate to `115200`.

After startup, the sketch resets and configures the CC1101 and prints several register values for diagnostics.

These include:

```text
PARTNUM
VERSION
IOCFG2
PKTCTRL0
MDMCFG4
MDMCFG3
MDMCFG2
AGCCTRL2
AGCCTRL1
AGCCTRL0
FREND0
MARCSTATE
```

The receiver is then enabled automatically.

---

## Wi-Fi

Wi-Fi is intentionally disabled during startup:

```cpp
WiFi.mode(WIFI_OFF);
WiFi.forceSleepBegin();
```

This reduces background activity and helps keep RF measurements and microsecond-level timing as stable as possible.

The ESP8266 is therefore used mainly as a microcontroller for:

- CC1101 control
- RF pulse measurement
- PWM decoding
- RF transmission
- Serial interaction

---

## Receiver Architecture

The RX data path is roughly:

```text
CC1101 GDO2
     │
     ▼
GPIO Interrupt
     │
     ▼
Raw Ring Buffer
     │
     ▼
Glitch Filter
     │
     ▼
Sync Detection
     │
     ▼
24-Bit Decoder
     │
     ▼
Frame Validation
     │
     ▼
Serial Output
```

The interrupt handler only records pulse timing information.

The actual decoding logic runs outside the interrupt context.

---

## Main Loop

The main loop remains intentionally small:

```cpp
void loop() {
    serviceReceiver();
    serviceSerial();
    yield();
}
```

It continuously:

- processes received RF pulses,
- handles Serial Monitor commands,
- services ESP8266 background tasks.

---

## Important Configuration Values

The most relevant parameters can be adjusted directly in the sketch:

```cpp
const uint32_t METZLER_CODE = 0x0D8C78UL;
const uint32_t RF_FREQ_KHZ = 433950UL;

const uint8_t TX_PA_LEVEL = 0x60;

const uint16_t DEFAULT_TX_REPEATS = 52;

const uint32_t TX_T_US = 303;

const uint32_t GLITCH_MAX_US = 120;

const uint8_t CONFIRM_FRAMES = 3;
```

These values control, among other things:

- default transmitter code,
- RF frequency,
- TX power configuration,
- number of transmitted frames,
- PWM timing,
- glitch rejection,
- required number of matching RX frames.

---

## Troubleshooting

### CC1101 reset fails

If the Serial Monitor shows:

```text
CC1101 RESET FEHLER
```

check the SPI wiring and power supply.

In particular:

```text
VCC
GND
SCK
MISO
MOSI
CSN
```

---

### The transmitter is very close but reception is unreliable

Toggle near-field mode:

```text
g
```

The Serial Monitor should report:

```text
RX Gain: NAHFELD
```

This reduces receiver gain and can improve decoding at very short distances.

---

### The original receiver does not react to transmitted codes

Toggle TX polarity:

```text
i
```

and transmit again:

```text
t
```

---

### Inspecting unknown transmitters

Enable output of other valid 24-bit codes:

```text
v
```

This allows the sketch to report valid frames that do not match the configured Metzler code.

---

### RX ring buffer overflow

If the sketch reports:

```text
WARNUNG: RX Ringbuffer Overflow
```

too many RF edges were received before the main loop could process them.

Possible causes include:

- excessive RF noise,
- strong nearby transmitters,
- unsuitable receiver gain,
- interference on the selected frequency.

Trying near-field mode may help when testing very close to a transmitter.

---

## Known Tested Device

The tested Metzler transmitter device is:

```text
HE-MT10V02
```


---

## Project Status

The sketch currently implements both **RX and TX** for the tested Metzler 24-bit PWM protocol.

It also includes diagnostic and development features for:

- analyzing received RF frames,
- monitoring RSSI,
- switching receiver gain,
- testing TX polarity,
- transmitting custom 24-bit codes,
- inspecting other compatible RF transmitters.

---

## Disclaimer

This project was developed around a specific tested Metzler RF transmitter and its observed waveform.

Other Metzler devices may use different:

- RF frequencies,
- device codes,
- timings,
- protocols,
- modulation parameters.

Use the project only with devices and radio frequencies you are permitted to operate in your region.

/*
 *  Project     Adalight FastLED ESP32
 *  @author     Chris Dressel
 *  @link       github.com/christoros/Adalight-FastLED-ESP32
 *  @license    LGPL - Copyright (c) 2017 David Madison
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <Arduino.h>
#include <FastLED.h>

// --- General Settings
const uint16_t Num_Leds = 120;  // Strip length
const uint8_t Brightness = 255; // Maximum brightness

// --- FastLED Settings
#define LED_TYPE WS2812B   // LED strip type for FastLED
#define COLOR_ORDER GRB    // Color order for FastLED
#define PIN_DATA 13        // Use GPIO13 (not a strapping pin) on ESP32

// --- Serial Settings
const unsigned long SerialSpeed = 921600; // Serial port speed (must match Prismatik)
const uint16_t SerialTimeout = 60;        // Timeout (in seconds) before LEDs are cleared

// --- Optional Settings
#define SERIAL_FLUSH
// #define CLEAR_ON_START

CRGB leds[Num_Leds];
uint8_t *ledsRaw = (uint8_t *)leds;

// --- Adalight header settings
const uint8_t magic[] = {'A', 'd', 'a'};
#define MAGICSIZE sizeof(magic)
#define HICHECK   (MAGICSIZE)
#define LOCHECK   (MAGICSIZE + 1)
#define CHECKSUM  (MAGICSIZE + 2)

enum processModes_t { Header, Data } mode = Header;
int16_t c;            // current byte (must support -1 when no data)
uint16_t outPos;      // current position in LED data array
uint32_t bytesRemaining;  // remaining bytes expected for the LED data
unsigned long t, lastByteTime, lastAckTime;  // time tracking variables

// Function declarations
void headerMode();
void dataMode();
void timeouts();

// Define SERIAL_FLUSH macro to clear any extra serial data
#ifdef SERIAL_FLUSH
  #undef SERIAL_FLUSH
  #define SERIAL_FLUSH while(Serial.available() > 0) { Serial.read(); }
#else
  #define SERIAL_FLUSH
#endif

void setup() {
  Serial.begin(SerialSpeed);  // Initialize the main serial port
  // Add the LED strip on GPIO13 with the chosen type and color order.
  FastLED.addLeds<LED_TYPE, PIN_DATA, COLOR_ORDER>(leds, Num_Leds).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(Brightness);

  #ifdef CLEAR_ON_START
    FastLED.show();
  #endif

  // Send an initial acknowledgement to Prismatik
  Serial.print("Ada\n");
  lastByteTime = lastAckTime = millis();
}

void loop() {
  t = millis();

  // If data is available on the serial port, process it byte-by-byte
  if ((c = Serial.read()) >= 0) {
    lastByteTime = lastAckTime = t; // Reset timeouts on incoming data

    // Process based on current mode: header or data
    switch (mode) {
      case Header:
        headerMode();
        break;
      case Data:
        dataMode();
        break;
    }
  }
  else {
    timeouts();
  }
}

void headerMode() {
  // Static variables to track header state across calls
  static uint8_t headPos = 0, hi = 0, lo = 0, chk = 0;

  if (headPos < MAGICSIZE) {
    // Validate the magic header bytes
    if (c == magic[headPos]) {
      headPos++;
    } else {
      headPos = 0;
    }
  } else {
    // After magic word, process the next three bytes: high byte, low byte, checksum
    switch (headPos) {
      case HICHECK:
        hi = c;
        headPos++;
        break;
      case LOCHECK:
        lo = c;
        headPos++;
        break;
      case CHECKSUM:
        chk = c;
        // Validate checksum: (hi XOR lo XOR 0x55)
        if (chk == (hi ^ lo ^ 0x55)) {
          // Compute the expected data length (number of LEDs * 3 bytes per LED)
          bytesRemaining = 3L * (256L * (long)hi + (long)lo + 1L);
          outPos = 0;
          // Clear the LED array before receiving new data
          memset(leds, 0, Num_Leds * sizeof(CRGB));
          mode = Data; // Switch to data mode
        }
        headPos = 0; // Reset header tracking regardless of checksum result
        break;
    }
  }
}

void dataMode() {
  // Load incoming bytes into the LED array until complete
  if (outPos < sizeof(leds)) {
    ledsRaw[outPos++] = c;
  }
  bytesRemaining--;

  // When all expected data bytes have been received...
  if (bytesRemaining == 0) {
    mode = Header; // Reset mode to look for the next header
    FastLED.show();  // Update the LED strip
    SERIAL_FLUSH;    // Clear any extra data from the serial buffer
  }
}

void timeouts() {
  // If no data has been received recently, send an ACK every second.
  if ((t - lastAckTime) >= 1000) {
    Serial.print("Ada\n");
    lastAckTime = t;

    // If extended silence is detected, clear the LED display.
    if (SerialTimeout != 0 && (t - lastByteTime) >= (uint32_t)SerialTimeout * 1000) {
      memset(leds, 0, Num_Leds * sizeof(CRGB));
      FastLED.show();
      mode = Header;
      lastByteTime = t;
    }
  }
}

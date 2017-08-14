//Huge bits of this code are copypasted. Sorry.

// All the mcufriend.com UNO shields have the same pinout.
// i.e. control pins A0-A4.  Data D2-D9.  microSD D10-D13.
// Touchscreens are normally A1, A2, D7, D6 but the order varies
//
// This demo should work with most Adafruit TFT libraries
// If you are not using a shield,  use a full Adafruit constructor()
// e.g. Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
//#include <Adafruit_TFTLCD.h>
//Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif



uint16_t g_identifier;

extern const uint8_t hanzi[];

void setup(void) {
    Serial.begin(1000000);
    tft.begin(0x1520);
    tft.setRotation(1);
    tft.fillScreen(BLACK);
}

#define uart_buf_pixel_count 20
#define uart_buf_size uart_buf_pixel_count * 2

byte count = 0;

byte is_first = 1;
byte img[uart_buf_size];

long total_pixels_count = 0;

void loop(void) {
    if (Serial.available() >= uart_buf_size) {
      Serial.readBytes(img, uart_buf_size);
      tft.pushColors((img), uart_buf_pixel_count, is_first);
      is_first = 0;
      total_pixels_count += uart_buf_pixel_count;
      if (total_pixels_count == 76800) {
        is_first = 1;
        total_pixels_count = 0;
      }
    }
}


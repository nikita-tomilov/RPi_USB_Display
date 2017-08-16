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

void setup(void) {
    //Serial.begin(230400);
    //Serial.begin(1000000);
    //Serial.begin(500000);
    Serial.begin(921600);
    tft.begin(0x1520);
    tft.setRotation(1);
    tft.fillScreen(WHITE);

    delay(100);
}

#define PIXELS_PER_CHUNK 8
#define BYTES_PER_CHUNK PIXELS_PER_CHUNK * 2

#define CHUNKS_PER_READ 1
#define BYTES_PER_READ ((4 + BYTES_PER_CHUNK) * CHUNKS_PER_READ)
 
#define uart_buf_size BYTES_PER_READ


byte img[uart_buf_size];


volatile uint16_t newx = 0, newy = 0;

uint16_t color = 0;



void loop(void) {
    if (Serial.available() >= uart_buf_size) {
      Serial.readBytes(img, uart_buf_size);
           newx = (((int16_t)(img[0])) << 8) | (int16_t)(img[1]);
           newy = (((int16_t)(img[2])) << 8) | (int16_t)(img[3]);

           tft.setAddrWindow(newx, newy, newx+PIXELS_PER_CHUNK-1, newy+1);
           tft.pushColors(img + 4, PIXELS_PER_CHUNK, true);     
    }
}


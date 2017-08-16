//Huge bits of this code are copypasted. Sorry.

#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#include <SPI.h>          // f.k. for Arduino-1.5.2

#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

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

//touchscreen stuff
#include <TouchScreen.h>
uint8_t YP = A1;  // must be an analog pin, use "An" notation!
uint8_t XM = A2;  // must be an analog pin, use "An" notation!
uint8_t YM = 7;   // can be a digital pin
uint8_t XP = 6;   // can be a digital pin
uint8_t SwapXY = 0;

/*uint16_t TS_LEFT = 920;
uint16_t TS_RT  = 150;
uint16_t TS_TOP = 940;
uint16_t TS_BOT = 120;*/

//my calibr
uint16_t TS_LEFT = 863;
uint16_t TS_RT  = 130;
uint16_t TS_TOP = 108;
uint16_t TS_BOT = 900;

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint tp;
#define SWAP(a, b) {uint16_t tmp = a; a = b; b = tmp;}
#define MINPRESSURE 5
#define MAXPRESSURE 1000

void setup(void) {

    uint16_t tmp;
    //swap for landscape mode
    tmp = TS_LEFT, TS_LEFT = TS_BOT, TS_BOT = TS_RT, TS_RT = TS_TOP, TS_TOP = tmp;
    ts = TouchScreen(XP, YP, XM, YM, 300); 
    
    tft.begin(0x1520);
    tft.setRotation(1);
    tft.fillScreen(WHITE);

    //Serial.begin(9600);
    //Serial.begin(230400);
    //Serial.begin(1000000);
    //Serial.begin(500000);
    Serial.begin(921600);
    delay(100);
    //touchtest();
}

#define PIXELS_PER_CHUNK 8
#define BYTES_PER_CHUNK PIXELS_PER_CHUNK * 2

#define CHUNKS_PER_READ 1
#define BYTES_PER_READ ((4 + BYTES_PER_CHUNK) * CHUNKS_PER_READ)
 
#define uart_buf_size BYTES_PER_READ


byte img[uart_buf_size];


volatile uint16_t newx = 0, newy = 0;

uint16_t color = 0;

TSPoint oldtp;
bool oldtp_printed = true;

void checktouch(void) {
    tp = ts.getPoint();
    if ((tp.z > MINPRESSURE) && (tp.z < MAXPRESSURE)) {
      SWAP(tp.x, tp.y)
      oldtp = tp;
      oldtp_printed = false;
    }
}

void loop(void) {
    if (Serial.available() >= uart_buf_size) {
      Serial.readBytes(img, uart_buf_size);

      if ((img[0] == 0xFF) && (img[1] == 0xFF)) {
        //request for touch
        //tft.fillRect(0, 0, 100, 100, RED);
        for (int i = 0; i < 20; i++) {
          checktouch();
        }
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        pinMode(XP, OUTPUT);
        pinMode(YM, OUTPUT);
        if (!oldtp_printed) {
            uint16_t xpos, ypos;  //screen coordinates

            xpos = map(oldtp.x, TS_RT, TS_LEFT, 0, tft.width());
            ypos = map(oldtp.y, TS_BOT, TS_TOP, 0, tft.height());
            uint8_t buf[6];
            tft.fillRect(xpos-2, ypos-2, 5, 5, BLACK);
            tft.fillRect(xpos-1, ypos-1, 3, 3, WHITE);
            buf[0] = (uint8_t)((xpos >> 8) & 0xFF);
            buf[1] = (uint8_t)((xpos) & 0xFF);
            buf[2] = (uint8_t)((ypos >> 8) & 0xFF);
            buf[3] = (uint8_t)((ypos) & 0xFF);
            buf[4] = (uint8_t)((oldtp.z >> 8) & 0xFF);
            buf[5] = (uint8_t)((oldtp.z) & 0xFF);
            Serial.write(buf, 6);
            oldtp_printed = true;
        } else {
          Serial.println("Nothing");
        }
        
      } else {
        newx = (((int16_t)(img[0])) << 8) | (int16_t)(img[1]);
        newy = (((int16_t)(img[2])) << 8) | (int16_t)(img[3]);
        
        tft.setAddrWindow(newx, newy, newx+PIXELS_PER_CHUNK-1, newy+1);
        tft.pushColors(img + 4, PIXELS_PER_CHUNK, true);     
      }
      
    }
    
}


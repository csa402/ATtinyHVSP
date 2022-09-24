//test ports pour platine HVSP

#include <avr/io.h>       // for GPIO
#include <avr/pgmspace.h> // to store data in programm memory
#include <util/delay.h>   // for delays


/*
#define RST_PIN   PB5 //13 // 12V !RESET                       Pin 1 of target device
#define SCI_PIN   PB4 //12 // Serial Clock Input (SCI)         Pin 2 of target device
#define SDO_PIN   PB3 //11 // Serial Data Output (SDO)         Pin 7 of target device
#define SII_PIN   PB2 //10 // Serial Instruction Input (SII)   Pin 6 of target device
#define SDI_PIN   PB1 //9  // Serial Data Input (SDI)          Pin 5 of target device
#define VCC_PIN   PB0 //8  // Target VCC                       Pin 8 of target device
#define I2C_SCL   PC5 // I2C Serial Clock (SCK)
#define I2C_SDA   PC4 // I2C Serial Data (SDA)
#define BUTTON    PD7 // OK-Button
*/

#define PIN PB0

const char TitleScreen[] PROGMEM = "Virtual  PIN : ";
const char TitlePb[] PROGMEM = "PB";
const char TitlePd[] PROGMEM = "PD";
const char TitleScreen2[] PROGMEM = "Physical PIN : ";

void setup() {
OLED_init();                            // setup I2C OLED
OLED_clearScreen(); 
OLED_setCursor(0,0);
OLED_printPrg(TitleScreen);

switch (PIN) {
case PB0:
         DDRB  |= (1<<PIN);
         OLED_printPrg(TitlePb);
         OLED_printHex(0x00);
         OLED_setCursor(0,2);
         OLED_printPrg(TitleScreen2);
         OLED_printHex(0x08);
         break;
case PB1:
         DDRB  |= (1<<PIN);
         OLED_printPrg(TitlePb);
         OLED_printHex(0x01);
         OLED_setCursor(0,2);
         OLED_printPrg(TitleScreen2);
         OLED_printHex(0x09);
         break;
case PB2:
         DDRB  |= (1<<PIN);
         OLED_printPrg(TitlePb);
         OLED_printHex(0x02);
         OLED_setCursor(0,2);
         OLED_printPrg(TitleScreen2);
         OLED_printHex(0x10);
         break;
case PB3:
         DDRB  |= (1<<PIN);
         OLED_printPrg(TitlePb);
         OLED_printHex(0x03);
         OLED_setCursor(0,2);
         OLED_printPrg(TitleScreen2);
         OLED_printHex(0x11);
         break;
case PB4:
         DDRB  |= (1<<PIN);
         OLED_printPrg(TitlePb);
         OLED_printHex(0x04);
         OLED_setCursor(0,2);
         OLED_printPrg(TitleScreen2);
         OLED_printHex(0x12);
         break;
         
case PB5:
         DDRB  |= (1<<PIN);
         OLED_printPrg(TitlePb);
         OLED_printHex(0x05);
         OLED_setCursor(0,2);
         OLED_printPrg(TitleScreen2);
         OLED_printHex(0x13);
         break;
         
case PD7:
         DDRD |= (1<<PIN);
         OLED_printPrg(TitlePd);
         OLED_printHex(0x07);
         OLED_setCursor(0,2);
         OLED_printPrg(TitleScreen2);
         OLED_printHex(0x07);
         break;
default: 
         break;
}

 
 

 
 
}

void loop() {
     PORTB &= ~(1<<PIN);
     delay(100);
     PORTB |=  (1<<PIN);
     delay(100);
     PORTD &= ~(1<<PIN);
     delay(100);
     PORTD |=  (1<<PIN);
     delay(100);
}

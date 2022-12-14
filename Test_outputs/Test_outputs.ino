//test ports pour platine HVSP
//test des entrées/sortie et I2C_Scanner
//Oled Screen 0x3C
//Ram 0x50

#include <avr/io.h>       // for GPIO
#include <avr/pgmspace.h> // to store data in programm memory
#include <util/delay.h>   // for delays
#include <Wire.h>

/* Rappel connections : 
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
int boucle;

#define PIN PB0

#define I2C_SCL   PC5 // I2C Serial Clock (SCK)
#define I2C_SDA   PC4 // I2C Serial Data (SDA)

// ===================================================================================
// I2C Implementation
// ===================================================================================

// I2C macros
#define I2C_SDA_HIGH()  DDRC &= ~(1<<I2C_SDA) // release SDA   -> pulled HIGH by resistor
#define I2C_SDA_LOW()   DDRC |=  (1<<I2C_SDA) // SDA as output -> pulled LOW  by MCU
#define I2C_SCL_HIGH()  DDRC &= ~(1<<I2C_SCL) // release SCL   -> pulled HIGH by resistor
#define I2C_SCL_LOW()   DDRC |=  (1<<I2C_SCL) // SCL as output -> pulled LOW  by MCU
#define I2C_DELAY()     asm("lpm")            // delay 3 clock cycles
#define I2C_CLOCKOUT()  I2C_SCL_HIGH();I2C_DELAY();I2C_SCL_LOW()  // clock out

// I2C init function
void I2C_init(void) {
  DDRC  &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // pins as input (HIGH-Z) -> lines released
  PORTC &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // should be LOW when as ouput
}

// I2C transmit one data byte to the slave, ignore ACK bit, no clock stretching allowed
void I2C_write(uint8_t data) {
  for(uint8_t i = 8; i; i--, data<<=1) {  // transmit 8 bits, MSB first
    (data & 0x80) ? (I2C_SDA_HIGH()) : (I2C_SDA_LOW());  // SDA HIGH if bit is 1
    I2C_CLOCKOUT();                       // clock out -> slave reads the bit
  }
  I2C_DELAY();                            // delay 3 clock cycles
  I2C_SDA_HIGH();                         // release SDA for ACK bit of slave
  I2C_CLOCKOUT();                         // 9th clock pulse is for the ignored ACK bit
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  I2C_SDA_LOW();                          // start condition: SDA goes LOW first
  I2C_SCL_LOW();                          // start condition: SCL goes LOW second
  I2C_write(addr);                        // send slave address
}

// I2C stop transmission
void I2C_stop(void) {
  I2C_SDA_LOW();                          // prepare SDA for LOW to HIGH transition
  I2C_SCL_HIGH();                         // stop condition: SCL goes HIGH first
  I2C_SDA_HIGH();                         // stop condition: SDA goes HIGH second
}

// ===================================================================================
// OLED Implementation
// ===================================================================================

// OLED definitions
#define OLED_ADDR       0x78              // OLED write address
#define OLED_CMD_MODE   0x00              // set command mode
#define OLED_DAT_MODE   0x40              // set data mode
#define OLED_INIT_LEN   9                 // length of init command array

// OLED 5x8 pixels character set
const uint8_t OLED_FONT[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62,
  0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, 0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x00, 0xA0, 0x60, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x60, 0x60, 0x00, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00,
  0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x18, 0x14, 0x12, 0x7F, 0x10,
  0x27, 0x45, 0x45, 0x45, 0x39, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03,
  0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x36, 0x36, 0x00, 0x00,
  0x00, 0x56, 0x36, 0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x51, 0x09, 0x06, 0x32, 0x49, 0x59, 0x51, 0x3E,
  0x7C, 0x12, 0x11, 0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
  0x7F, 0x41, 0x41, 0x22, 0x1C, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09, 0x09, 0x09, 0x01,
  0x3E, 0x41, 0x49, 0x49, 0x7A, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00,
  0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40, 0x40,
  0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x3E, 0x41, 0x41, 0x41, 0x3E,
  0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46,
  0x46, 0x49, 0x49, 0x49, 0x31, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x3F, 0x40, 0x40, 0x40, 0x3F,
  0x1F, 0x20, 0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63,
  0x07, 0x08, 0x70, 0x08, 0x07, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x7F, 0x41, 0x41, 0x00,
  0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x04, 0x02, 0x01, 0x02, 0x04,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x01, 0x02, 0x04, 0x00, 0x20, 0x54, 0x54, 0x54, 0x78,
  0x7F, 0x48, 0x44, 0x44, 0x38, 0x38, 0x44, 0x44, 0x44, 0x20, 0x38, 0x44, 0x44, 0x48, 0x7F,
  0x38, 0x54, 0x54, 0x54, 0x18, 0x08, 0x7E, 0x09, 0x01, 0x02, 0x18, 0xA4, 0xA4, 0xA4, 0x7C,
  0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00,
  0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,
  0x7C, 0x08, 0x04, 0x04, 0x78, 0x38, 0x44, 0x44, 0x44, 0x38, 0xFC, 0x24, 0x24, 0x24, 0x18,
  0x18, 0x24, 0x24, 0x18, 0xFC, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x48, 0x54, 0x54, 0x54, 0x20,
  0x04, 0x3F, 0x44, 0x40, 0x20, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x1C, 0x20, 0x40, 0x20, 0x1C,
  0x3C, 0x40, 0x30, 0x40, 0x3C, 0x44, 0x28, 0x10, 0x28, 0x44, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C,
  0x44, 0x64, 0x54, 0x4C, 0x44, 0x08, 0x36, 0x41, 0x41, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00,
  0x00, 0x41, 0x41, 0x36, 0x08, 0x08, 0x04, 0x08, 0x10, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// OLED init settings
const uint8_t OLED_INIT_CMD[] PROGMEM = {
  0xC8, 0xA1,   // flip screen
  0xA8, 0x1F,   // set multiplex ratio
  0xDA, 0x02,   // set com pins hardware configuration
  0x8D, 0x14,   // set DC-DC enable
  0xAF          // display on
};

// OLED variables
uint8_t OLED_x, OLED_y;                   // current cursor position

// OLED init function
void OLED_init(void) {
  I2C_init();                             // initialize I2C first
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  for(uint8_t i = 0; i < OLED_INIT_LEN; i++)
    I2C_write(pgm_read_byte(&OLED_INIT_CMD[i])); // send the command bytes
  I2C_stop();                             // stop transmission
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  I2C_write(xpos & 0x0F);                 // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));          // set high nibble of start column
  I2C_write(0xB0 | (ypos & 0x07));        // set start page
  I2C_stop();                             // stop transmission
  OLED_x = xpos; OLED_y = ypos;           // set the cursor variables
}

// OLED clear line
void OLED_clearLine(uint8_t line) {
  OLED_setCursor(0, line);                // set cursor to line start
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  for(uint8_t i=128; i; i--) I2C_write(0x00); // clear the line
  I2C_stop();                             // stop transmission
}

// OLED clear screen
void OLED_clearScreen(void) {
  for(uint8_t i=0; i<4; i++)              // 4 lines
    OLED_clearLine(i);                    // clear line
}

// OLED print a single character
void OLED_printChar(char c) {
  uint16_t ptr = c - 32;                  // character pointer
  ptr += ptr << 2;                        // -> ptr = (ch - 32) * 5;
  I2C_write(0x00);                        // write space between characters
  for(uint8_t i=5 ; i; i--) I2C_write(pgm_read_byte(&OLED_FONT[ptr++]));
  OLED_x += 6;                            // update cursor
  if(OLED_x > 122) {                      // line end ?
    I2C_stop();                           // stop data transmission
    OLED_setCursor(0,++OLED_y);           // set next line start
    I2C_start(OLED_ADDR);                 // start transmission to OLED
    I2C_write(OLED_DAT_MODE);             // set data mode
  }
}

// OLED print a string from program memory
void OLED_printPrg(const char* p) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  char ch = pgm_read_byte(p);             // read first character from program memory
  while(ch) {                             // repeat until string terminator
    OLED_printChar(ch);                   // print character on OLED
    ch = pgm_read_byte(++p);              // read next character
  }
  I2C_stop();                             // stop transmission
}

// OLED convert byte nibble into hex character and prints it
void OLED_printNibble(uint8_t nibble) {
  char c;
  if(nibble <= 9)  c = '0' + nibble;
  else             c = 'A' + nibble - 10;
  OLED_printChar(c);
}

// OLED print byte as hex
void OLED_printHex(uint8_t value) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  OLED_printNibble(value >> 4);           // print high nibble
  OLED_printNibble(value & 0x0F);         // print low nibble
  I2C_stop();                             // stop transmission
}




const char TitleScreen[] PROGMEM = "Virtual  PIN : ";
const char TitlePb[] PROGMEM = "PB";
const char TitlePd[] PROGMEM = "PD";
const char TitleScreen2[] PROGMEM = "Physical PIN : ";
const char BaudScreen[]   PROGMEM = "I2Cscanner 115200 bds";

void setup() {
OLED_init();                            // setup I2C OLED
OLED_clearScreen(); 
OLED_setCursor(0,0);
OLED_printPrg(TitleScreen);
Serial.begin(9600);
Serial.println("\nI2C Scanner :");


switch (PIN) {
case PB0:
         DDRB  |= (1<<PIN);
         OLED_printPrg(TitlePb);
         OLED_printHex(0x00);
         OLED_setCursor(0,1);
         OLED_printPrg(TitleScreen2);
         OLED_printHex(0x08);
         OLED_setCursor(0,3);
         OLED_printPrg(BaudScreen);
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
  byte error, address;
  int nDevices;
  

switch (boucle){
  case 10:
          Serial.println("Scanning...");
          nDevices = 0;
          for(address = 1; address < 127; address++ )
          { 
          Wire.beginTransmission(address);
          error = Wire.endTransmission();
          if (error == 0)
          {
          Serial.print("I2C device found at address 0x");
          if (address<16) 
          Serial.print("0");
          Serial.print(address,HEX);
          Serial.println("  !");
          nDevices++;
          }
          else if (error==4) 
          {
          Serial.print("Unknow error at address 0x");
          if (address<16) 
          Serial.print("0");
          Serial.println(address,HEX);
          }    
          }
          if (nDevices == 0)
          Serial.println("No I2C devices found\n");
          else
          Serial.println("done\n");
          boucle=0;
          break;
   
   default:
          boucle++;
}
 
  switch (PIN) {
  case PD7 :
           PORTD &= ~(1<<PIN);
           delay(100);
           PORTD |=  (1<<PIN);
           delay(100);
           break;
  default:
           PORTB &= ~(1<<PIN);
           delay(100);
           PORTB |=  (1<<PIN);
           delay(100);
           break;
  
}

}

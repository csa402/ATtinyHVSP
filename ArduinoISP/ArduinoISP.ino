//AVRISP Specifique pour platine HVSPreset


#define I2C_SCL   PC5 // I2C Serial Clock (SCK)
#define I2C_SDA   PC4 // I2C Serial Data (SDA)

#define I2C_SDA_HIGH()  DDRC &= ~(1<<I2C_SDA) // release SDA   -> pulled HIGH by resistor
#define I2C_SDA_LOW()   DDRC |=  (1<<I2C_SDA) // SDA as output -> pulled LOW  by MCU
#define I2C_SCL_HIGH()  DDRC &= ~(1<<I2C_SCL) // release SCL   -> pulled HIGH by resistor
#define I2C_SCL_LOW()   DDRC |=  (1<<I2C_SCL) // SCL as output -> pulled LOW  by MCU
#define I2C_DELAY()     asm("lpm")            // delay 3 clock cycles
#define I2C_CLOCKOUT()  I2C_SCL_HIGH();I2C_DELAY();I2C_SCL_LOW()  // clock out



#include "Arduino.h"
#undef SERIAL


#define PROG_FLICKER true

// Configure SPI clock (in Hz).
// E.g. for an ATtiny @ 128 kHz: the datasheet states that both the high and low
// SPI clock pulse must be > 2 CPU cycles, so take 3 cycles i.e. divide target
// f_cpu by 6:
//     #define SPI_CLOCK            (128000/6)
//
// A clock slow enough for an ATtiny85 @ 1 MHz, is a reasonable default:

#define SPI_CLOCK 		(1000000/6)


// Select hardware or software SPI, depending on SPI clock.
// Currently only for AVR, for other architectures (Due, Zero,...), hardware SPI
// is probably too fast anyway.

#if defined(ARDUINO_ARCH_AVR)

#if SPI_CLOCK > (F_CPU / 128)
#define USE_HARDWARE_SPI
#endif

#endif

// Configure which pins to use:

#define VCC       8
#define RESET     13 // Use pin 10 to reset the target rather than SS
#define PIN_MOSI	9
#define PIN_MISO	10
#define PIN_SCK		11
#define PIN2      12

// By default, use hardware SPI pins:
#ifndef PIN_MOSI
#define PIN_MOSI 	MOSI
#endif

#ifndef PIN_MISO
#define PIN_MISO 	MISO
#endif

#ifndef PIN_SCK
#define PIN_SCK 	SCK
#endif

// Force bitbanged SPI if not using the hardware SPI pins:
#if (PIN_MISO != MISO) ||  (PIN_MOSI != MOSI) || (PIN_SCK != SCK)
#undef USE_HARDWARE_SPI
#endif


// Configure the serial port to use.
//
// Prefer the USB virtual serial port (aka. native USB port), if the Arduino has one:
//   - it does not autoreset (except for the magic baud rate of 1200).
//   - it is more reliable because of USB handshaking.
//
// Leonardo and similar have an USB virtual serial port: 'Serial'.
// Due and Zero have an USB virtual serial port: 'SerialUSB'.
//
// On the Due and Zero, 'Serial' can be used too, provided you disable autoreset.
// To use 'Serial': #define SERIAL Serial

#ifdef SERIAL_PORT_USBVIRTUAL
#define SERIAL SERIAL_PORT_USBVIRTUAL
#else
#define SERIAL Serial
#endif


// Configure the baud rate:

#define BAUDRATE	19200
// #define BAUDRATE	115200
// #define BAUDRATE	1000000


#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //ok it is a space...

void pulse(int pin, int times);

#ifdef USE_HARDWARE_SPI
#include "SPI.h"
#else

#define SPI_MODE0 0x00

#if !defined(ARDUINO_API_VERSION) || ARDUINO_API_VERSION != 10001 // A SPISettings class is declared by ArduinoCore-API 1.0.1
class SPISettings {
  public:
    // clock is in Hz
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) : clockFreq(clock) {
      (void) bitOrder;
      (void) dataMode;
    };

    uint32_t getClockFreq() const {
      return clockFreq;
    }

  private:
    uint32_t clockFreq;
};
#endif  // !defined(ARDUINO_API_VERSION)

class BitBangedSPI {
  public:
    void begin() {
      digitalWrite(PIN_SCK, LOW);
      digitalWrite(PIN_MOSI, LOW);
      pinMode(PIN_SCK, OUTPUT);
      pinMode(PIN_MOSI, OUTPUT);
      pinMode(PIN_MISO, INPUT);
    }

    void beginTransaction(SPISettings settings) {
      pulseWidth = (500000 + settings.getClockFreq() - 1) / settings.getClockFreq();
      if (pulseWidth == 0) {
        pulseWidth = 1;
      }
    }

    void end() {}

    uint8_t transfer(uint8_t b) {
      for (unsigned int i = 0; i < 8; ++i) {
        digitalWrite(PIN_MOSI, (b & 0x80) ? HIGH : LOW);
        digitalWrite(PIN_SCK, HIGH);
        delayMicroseconds(pulseWidth);
        b = (b << 1) | digitalRead(PIN_MISO);
        digitalWrite(PIN_SCK, LOW); // slow pulse
        delayMicroseconds(pulseWidth);
      }
      return b;
    }

  private:
    unsigned long pulseWidth; // in microseconds
};

static BitBangedSPI SPI;

#endif

const char TitleScreen[] PROGMEM =
"    AVRISP Mode:     "
"                     "
"Disconnect 12 Volts !";
  
void setup() {
  SERIAL.begin(BAUDRATE);
  pinMode(VCC,OUTPUT);
  pinMode(PIN2,OUTPUT);
  digitalWrite(VCC,HIGH);
  digitalWrite(PIN2,LOW);
  reset_target(false);
  OLED_init();
  OLED_clearScreen();
  OLED_setCursor(0,0);
  OLED_printPrg(TitleScreen);
}

int ISPError = 0;
int pmode = 0;
// address for reading and writing, set by 'U' command
unsigned int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr+1) )
typedef struct param {
  uint8_t devicecode;
  uint8_t revision;
  uint8_t progtype;
  uint8_t parmode;
  uint8_t polling;
  uint8_t selftimed;
  uint8_t lockbytes;
  uint8_t fusebytes;
  uint8_t flashpoll;
  uint16_t eeprompoll;
  uint16_t pagesize;
  uint16_t eepromsize;
  uint32_t flashsize;
}
parameter;

parameter param;

// this provides a heartbeat on pin 9, so you can tell the software is running.
uint8_t hbval = 128;
int8_t hbdelta = 8;
void heartbeat() {
  static unsigned long last_time = 0;
  unsigned long now = millis();
  if ((now - last_time) < 40) {
    return;
  }
  last_time = now;
  if (hbval > 192) {
    hbdelta = -hbdelta;
  }
  if (hbval < 32) {
    hbdelta = -hbdelta;
  }
  hbval += hbdelta;

}

static bool rst_active_high;

void reset_target(bool reset) {
  digitalWrite(RESET, ((reset && rst_active_high) || (!reset && !rst_active_high)) ? HIGH : LOW);
}

void loop(void) {
  // is pmode active?
  if (pmode) {

  } else {

  }
  // is there an error?
  if (ISPError) {

  } else {

  }

  // light the heartbeat LED
  heartbeat();
  if (SERIAL.available()) {
    avrisp();
  }
}

uint8_t getch() {
  while (!SERIAL.available());
  return SERIAL.read();
}
void fill(int n) {
  for (int x = 0; x < n; x++) {
    buff[x] = getch();
  }
}

#define PTIME 30
void pulse(int pin, int times) {
  do {
    digitalWrite(pin, HIGH);
    delay(PTIME);
    digitalWrite(pin, LOW);
    delay(PTIME);
  } while (times--);
}

void prog_lamp(int state) {
  if (PROG_FLICKER) {
   }
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  SPI.transfer(a);
  SPI.transfer(b);
  SPI.transfer(c);
  return SPI.transfer(d);
}

void empty_reply() {
  if (CRC_EOP == getch()) {
    SERIAL.print((char)STK_INSYNC);
    SERIAL.print((char)STK_OK);
  } else {
    ISPError++;
    SERIAL.print((char)STK_NOSYNC);
  }
}

void breply(uint8_t b) {
  if (CRC_EOP == getch()) {
    SERIAL.print((char)STK_INSYNC);
    SERIAL.print((char)b);
    SERIAL.print((char)STK_OK);
  } else {
    ISPError++;
    SERIAL.print((char)STK_NOSYNC);
  }
}

void get_version(uint8_t c) {
  switch (c) {
    case 0x80:
      breply(HWVER);
      break;
    case 0x81:
      breply(SWMAJ);
      break;
    case 0x82:
      breply(SWMIN);
      break;
    case 0x93:
      breply('S'); // serial programmer
      break;
    default:
      breply(0);
  }
}

void set_parameters() {
  // call this after reading parameter packet into buff[]
  param.devicecode = buff[0];
  param.revision   = buff[1];
  param.progtype   = buff[2];
  param.parmode    = buff[3];
  param.polling    = buff[4];
  param.selftimed  = buff[5];
  param.lockbytes  = buff[6];
  param.fusebytes  = buff[7];
  param.flashpoll  = buff[8];
  // ignore buff[9] (= buff[8])
  // following are 16 bits (big endian)
  param.eeprompoll = beget16(&buff[10]);
  param.pagesize   = beget16(&buff[12]);
  param.eepromsize = beget16(&buff[14]);

  // 32 bits flashsize (big endian)
  param.flashsize = buff[16] * 0x01000000
                    + buff[17] * 0x00010000
                    + buff[18] * 0x00000100
                    + buff[19];

  // AVR devices have active low reset, AT89Sx are active high
  rst_active_high = (param.devicecode >= 0xe0);
}

void start_pmode() {

  // Reset target before driving PIN_SCK or PIN_MOSI

  // SPI.begin() will configure SS as output, so SPI master mode is selected.
  // We have defined RESET as pin 10, which for many Arduinos is not the SS pin.
  // So we have to configure RESET as output here,
  // (reset_target() first sets the correct level)
  reset_target(false); //true
  pinMode(RESET, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));

  // See AVR datasheets, chapter "SERIAL_PRG Programming Algorithm":

  // Pulse RESET after PIN_SCK is low:
  digitalWrite(PIN_SCK, LOW);
  delay(20); // discharge PIN_SCK, value arbitrarily chosen
  reset_target(true); //false
  // Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
  // speeds above 20 KHz
  delayMicroseconds(100);
  reset_target(false); //true

  // Send the enable programming command:
  delay(50); // datasheet: must be > 20 msec
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  pmode = 1;
}

void end_pmode() {
  SPI.end();
  // We're about to take the target out of reset so configure SPI pins as input
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_SCK, INPUT);
  reset_target(true); //false
  pinMode(RESET, INPUT);
  pmode = 0;
}

void universal() {
  uint8_t ch;

  fill(4);
  ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
  breply(ch);
}

void flash(uint8_t hilo, unsigned int addr, uint8_t data) {
  spi_transaction(0x40 + 8 * hilo,
                  addr >> 8 & 0xFF,
                  addr & 0xFF,
                  data);
}
void commit(unsigned int addr) {
  if (PROG_FLICKER) {
    prog_lamp(LOW);
  }
  spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
  if (PROG_FLICKER) {
    delay(PTIME);
    prog_lamp(HIGH);
  }
}

unsigned int current_page() {
  if (param.pagesize == 32) {
    return here & 0xFFFFFFF0;
  }
  if (param.pagesize == 64) {
    return here & 0xFFFFFFE0;
  }
  if (param.pagesize == 128) {
    return here & 0xFFFFFFC0;
  }
  if (param.pagesize == 256) {
    return here & 0xFFFFFF80;
  }
  return here;
}


void write_flash(int length) {
  fill(length);
  if (CRC_EOP == getch()) {
    SERIAL.print((char) STK_INSYNC);
    SERIAL.print((char) write_flash_pages(length));
  } else {
    ISPError++;
    SERIAL.print((char) STK_NOSYNC);
  }
}

uint8_t write_flash_pages(int length) {
  int x = 0;
  unsigned int page = current_page();
  while (x < length) {
    if (page != current_page()) {
      commit(page);
      page = current_page();
    }
    flash(LOW, here, buff[x++]);
    flash(HIGH, here, buff[x++]);
    here++;
  }

  commit(page);

  return STK_OK;
}

#define EECHUNK (32)
uint8_t write_eeprom(unsigned int length) {
  // here is a word address, get the byte address
  unsigned int start = here * 2;
  unsigned int remaining = length;
  if (length > param.eepromsize) {
    ISPError++;
    return STK_FAILED;
  }
  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk(start, remaining);
  return STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(unsigned int start, unsigned int length) {
  // this writes byte-by-byte, page writing may be faster (4 bytes at a time)
  fill(length);
  prog_lamp(LOW);
  for (unsigned int x = 0; x < length; x++) {
    unsigned int addr = start + x;
    spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
    delay(45);
  }
  prog_lamp(HIGH);
  return STK_OK;
}

void program_page() {
  char result = (char) STK_FAILED;
  unsigned int length = 256 * getch();
  length += getch();
  char memtype = getch();
  // flash memory @here, (length) bytes
  if (memtype == 'F') {
    write_flash(length);
    return;
  }
  if (memtype == 'E') {
    result = (char)write_eeprom(length);
    if (CRC_EOP == getch()) {
      SERIAL.print((char) STK_INSYNC);
      SERIAL.print(result);
    } else {
      ISPError++;
      SERIAL.print((char) STK_NOSYNC);
    }
    return;
  }
  SERIAL.print((char)STK_FAILED);
  return;
}

uint8_t flash_read(uint8_t hilo, unsigned int addr) {
  return spi_transaction(0x20 + hilo * 8,
                         (addr >> 8) & 0xFF,
                         addr & 0xFF,
                         0);
}

char flash_read_page(int length) {
  for (int x = 0; x < length; x += 2) {
    uint8_t low = flash_read(LOW, here);
    SERIAL.print((char) low);
    uint8_t high = flash_read(HIGH, here);
    SERIAL.print((char) high);
    here++;
  }
  return STK_OK;
}

char eeprom_read_page(int length) {
  // here again we have a word address
  int start = here * 2;
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    SERIAL.print((char) ee);
  }
  return STK_OK;
}

void read_page() {
  char result = (char)STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  if (CRC_EOP != getch()) {
    ISPError++;
    SERIAL.print((char) STK_NOSYNC);
    return;
  }
  SERIAL.print((char) STK_INSYNC);
  if (memtype == 'F') {
    result = flash_read_page(length);
  }
  if (memtype == 'E') {
    result = eeprom_read_page(length);
  }
  SERIAL.print(result);
}

void read_signature() {
  if (CRC_EOP != getch()) {
    ISPError++;
    SERIAL.print((char) STK_NOSYNC);
    return;
  }
  SERIAL.print((char) STK_INSYNC);
  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
  SERIAL.print((char) high);
  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  SERIAL.print((char) middle);
  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  SERIAL.print((char) low);
  SERIAL.print((char) STK_OK);
}
//////////////////////////////////////////
//////////////////////////////////////////


////////////////////////////////////
////////////////////////////////////
void avrisp() {
  uint8_t ch = getch();
  switch (ch) {
    case '0': // signon
      ISPError = 0;
      empty_reply();
      break;
    case '1':
      if (getch() == CRC_EOP) {
        SERIAL.print((char) STK_INSYNC);
        SERIAL.print("AVR ISP");
        SERIAL.print((char) STK_OK);
      } else {
        ISPError++;
        SERIAL.print((char) STK_NOSYNC);
      }
      break;
    case 'A':
      get_version(getch());
      break;
    case 'B':
      fill(20);
      set_parameters();
      empty_reply();
      break;
    case 'E': // extended parameters - ignore for now
      fill(5);
      empty_reply();
      break;
    case 'P':
      if (!pmode) {
        start_pmode();
      }
      empty_reply();
      break;
    case 'U': // set address (word)
      here = getch();
      here += 256 * getch();
      empty_reply();
      break;

    case 0x60: //STK_PROG_FLASH
      getch(); // low addr
      getch(); // high addr
      empty_reply();
      break;
    case 0x61: //STK_PROG_DATA
      getch(); // data
      empty_reply();
      break;

    case 0x64: //STK_PROG_PAGE
      program_page();
      break;

    case 0x74: //STK_READ_PAGE 't'
      read_page();
      break;

    case 'V': //0x56
      universal();
      break;
    case 'Q': //0x51
      ISPError = 0;
      end_pmode();
      empty_reply();
      break;

    case 0x75: //STK_READ_SIGN 'u'
      read_signature();
      break;

    // expecting a command, not CRC_EOP
    // this is how we can get back in sync
    case CRC_EOP:
      ISPError++;
      SERIAL.print((char) STK_NOSYNC);
      break;

    // anything else we will return STK_UNKNOWN
    default:
      ISPError++;
      if (CRC_EOP == getch()) {
        SERIAL.print((char)STK_UNKNOWN);
      } else {
        SERIAL.print((char)STK_NOSYNC);
      }
  }
}

void I2C_init(void) {
  DDRC  &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // pins as input (HIGH-Z) -> lines released
  PORTC &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // should be LOW when as ouput
}

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

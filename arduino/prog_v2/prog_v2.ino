#include <Wire.h>

// todo: turn these all into port frobs so digitalWrite gets optimised out
#define A16 2
#define CE 3
#define OE 4
#define WE 5

// status LED defines
#define S1 6
#define S2 7

#define BUFLEN 32

#define NOP __asm__ __volatile__ ("nop")

// change this for different flash chips
#define MAX_ADDR 0x1ffff

void setup() {
  Serial.begin(57600);
  Serial.setTimeout(100);

  DDRD |= 0b11111100; // set status LEDs, control lines and A16 to outputs
  PORTD |= 0b00111000; // set control lines high
  
  Wire.begin();
  Wire.beginTransmission(0x20);
  Wire.write(0); // select IODIRA
  Wire.write(0); // all low, all outputs
  Wire.write(0); // sequential mode is enabled by default
  Wire.endTransmission(true);
}

/*
void set_waiting_for_cmd() { // in main loop, waiting for a read/write command
  digitalWrite(S1, HIGH);
  digitalWrite(S2, LOW);
}
void set_waiting_for_data() { // in read/write loop, waiting for more/ack/nak or a data block
  digitalWrite(S1, LOW);
  digitalWrite(S2, HIGH);
}
void set_talking_to_chip() { // reading or writing the flash
  digitalWrite(S1, HIGH);
  digitalWrite(S2, HIGH);
}
*/


// transfer protocol
// unexpected or incorrect commands are replied to with ?
// successful commands are replied to with !

/*
  command messages to programmer:
    read byte: r x+
    write byte: w x+ xx
    erase sector and set cursor: e x+
    set cursor: c x+
    read 4 bytes: R
    enter long write mode: l
    enter long read mode: L

  long mode:
    
*/

#define NORMAL 0
#define LONG_READ 1
#define LONG_WRITE 2

char SER_BUF[16];
uint32_t cursor = 0;
byte mode = NORMAL;

void loop() {
  switch (mode) {
    case NORMAL:
      short_mode();
      break;
    case LONG_WRITE:
      long_write_mode();
      break;
    case LONG_READ:
      long_read_mode();
      break;
    default:
      mode = NORMAL;
  }
}

void short_mode() {
  //while (!Serial.available());
  int len = Serial.readBytesUntil('\n', SER_BUF, 16);
  if (len == 0)
    return;

  /*
  Serial.print("got command ");
  Serial.write(SER_BUF, len);
  Serial.println();
  */
  switch (SER_BUF[0]) {
    case 'r':
      do_read_cmd(len);
      break;
    case 'w':
      do_write_cmd(len);
      break;
    case 'e':
      do_erase_s_cmd(len);
      break;
    case 'c':
      do_cursor_cmd(len);
      break;
    /*case 'R':
      do_big_read_cmd();
      break;*/
    case 'l':
      mode = LONG_WRITE;
      set_dout_mode();
      print_ok();
      break;
    case 'L':
      mode = LONG_READ;
      set_din_mode();
      print_ok();
      break;
    default:
      print_err();
  }
}

void long_write_mode() {
  int len = Serial.readBytes(SER_BUF, 16);
  if (len == 0) {
      mode = NORMAL;
      print_ok();
      return;
  }
  
  digitalWrite(S1, HIGH);
  for (int i = 0; i < 16; i++) {
    uint8_t d = SER_BUF[i];
    //print_hex_byte(d);
    program_cell(cursor, d);
    cursor++;
  }
  print_ok();
}
void long_read_mode() {
  int len = Serial.readBytes(SER_BUF, 1);
  if (len == 0) {
      mode = NORMAL;
      print_ok();
      return;
  }
  
  digitalWrite(S1, HIGH);
  for (int i = 0; i < 16; i++) {
    uint8_t d = read_cell(cursor);
    Serial.write(d);
    cursor++;
  }
}

void do_read_cmd(int len) {
  uint32_t addr = parse_hex_from_buffer(SER_BUF + 2, len - 2);
  if (addr > MAX_ADDR) {
    print_err();
    digitalWrite(S1, HIGH);
    return;
  }
  set_din_mode();
  uint8_t d = read_cell(addr);
  Serial.write('!');
  print_hex_byte(d);
  Serial.write('\n');
}
void do_write_cmd(int len) {
  uint32_t addr = parse_hex_from_buffer(SER_BUF + 2, len - 5); // -2 for cmd char and space, -3 for byte and space
  if (addr > MAX_ADDR) {
    print_err();
    return;
  }
  uint8_t d = parse_byte_from_buffer(SER_BUF + len - 2);
  set_dout_mode();
  program_cell(addr, d);
  print_ok();
}
void do_erase_s_cmd(int len) {
  uint32_t addr = parse_hex_from_buffer(SER_BUF + 2, len - 2);
  if (addr > MAX_ADDR || addr & 0xfff != 0) {
    print_err();
    return;
  }
  cursor = addr;
  set_dout_mode();
  erase_sector(addr);
  print_ok();
}
void do_cursor_cmd(int len) {
  uint32_t addr = parse_hex_from_buffer(SER_BUF + 2, len - 2);
  if (addr > MAX_ADDR) {
    print_err();
    return;
  }
  cursor = addr;
  print_ok();
}
/*void do_big_read_cmd() {
  Serial.write('!');
  set_din_mode();
  for (int i = 0; i < 4; i++) {
    uint8_t d = read_cell(cursor);
    print_hex_byte(d);
    cursor++;
  }
  Serial.write('\n');
}*/

void print_err() {
  print_status('?');
}
void print_ok() {
  print_status('!');
}
void print_status(char s) {
  Serial.write(s);
  Serial.write('\n');
}
void print_hex_byte(uint8_t d) {
  char lo = num_to_hex_char(d & 0xf);
  char hi = num_to_hex_char(d >> 4);
  Serial.write(hi);
  Serial.write(lo);
}

// no error checking tee hee
uint8_t hex_char_to_num(char c) {
  if (c >= 'a') {
    c -= 32;
  }
  c -= '0';
  if (c > 9) {
    c -= 7;
  }
  return c;
}
char num_to_hex_char(uint8_t n) {
  if (n < 10) {
    return n + '0';
  }
  else {
    return n + ('A' - 10);
  }
}

uint32_t parse_hex_from_buffer(char* buf, size_t len) {
  uint32_t acc = 0;
  for (int i = 0; i < len; i++) {
    acc <<= 4;
    acc += hex_char_to_num(buf[i]);
  }
  return acc;
}

uint8_t parse_byte_from_buffer(char* buf) {
  uint8_t hi = hex_char_to_num(buf[0]);
  uint8_t lo = hex_char_to_num(buf[1]);
  return (hi << 4) | lo;
}

// SAFETY: use set_din_mode before this
uint8_t read_cell(uint32_t addr) {
  set_addr(addr);
  PORTD &= 0b11100111;
  NOP; NOP; // just to make sure
  
  uint8_t d = read_data();
  
  PORTD |= 0b11000;
  return d;
}

// SAFETY: use set_dout_mode before this
void program_cell(uint32_t addr, uint8_t data) {
  write_a_d(0x5555, 0xaa);
  write_a_d(0x2aaa, 0x55);
  write_a_d(0x5555, 0xa0);
  write_a_d(addr, data);
  delayMicroseconds(25);
}

void erase_sector(uint32_t sax) {
  //uint32_t sax_masked = sax & (~0x0fff); // zero bits 0..=11

  write_a_d(0x5555, 0xaa);
  write_a_d(0x2aaa, 0x55);
  write_a_d(0x5555, 0x80);
  write_a_d(0x5555, 0xaa);
  write_a_d(0x2aaa, 0x55);
  write_a_d(sax, 0x30);
  
  delay(25);
}

void write_a_d(uint32_t addr, uint8_t data) {
  set_addr(addr);
  PORTD &= 0b11010111;
  write_data(data);
  PORTD |= 0b101000;
}

void set_addr(uint32_t addr) {
  //ext.writeGPIOAB((uint16_t)addr);
  Wire.beginTransmission(0x20);
  Wire.write(0x12); // select GPIOA
  Wire.write(addr & 0xff);
  Wire.write((addr >> 8) & 0xff);
  Wire.endTransmission(true);
  PORTD = (PORTD & ~0b100) | ((addr >> 14) & 0b100);
}


void set_dout_mode() {
  DDRB |= 0xf; // me when i twiddle bits
  DDRC |= 0xf;
}
// SAFETY: pb and pc 0..=3 must be set to outputs
void write_data(uint8_t data) {
  uint8_t low = data & 0xf;
  uint8_t high = data >> 4;

  PORTB = (PORTB & 0xf0) | low; // the joy of rmw
  PORTC = (PORTC & 0xf0) | high;
}

void set_din_mode() {
  DDRB &= 0xf0;
  DDRC &= 0xf0;
}
// SAFETY: pb and pc 0..=3 must be set to inputs
uint8_t read_data() {
  uint8_t low = PINB & 0xf;
  uint8_t high = PINC << 4;
  return high | low;
}

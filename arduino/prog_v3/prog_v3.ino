#include <SPI.h>

// todo: turn these all into port frobs so digitalWrite gets optimised out
#define A16 17
#define ADL 16
#define CE 8
#define OE 9
#define WE 10

// status LED defines
#define S1 15
#define S2 14

#define BUFLEN 32

#define NOP __asm__ __volatile__ ("nop")

// change this for different flash chips
#define MAX_ADDR 0x1ffff

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  SPI.begin();

  DDRB |= 0b101111; // set control/spi lines as outputs
  PORTB |= 0b111; // set control lines high

  DDRC |= 0b111100; // set the other control lines as outputs
  PORTC |= 0b100; // set ADL high

  /*long start = micros();
  for (unsigned int i = 0; i < 0x8000; i++) {
    set_addr(i);
  }
  long dur = micros() - start;
  Serial.println(dur);*/

  /*set_dout_mode();
  digitalWrite(2, HIGH);
  
  while (1) {
    int len = Serial.readBytesUntil('\n', SER_BUF, 16);
    if (len == 0)
      continue;

    byte d = parse_byte_from_buffer(SER_BUF);
    Serial.println(d, HEX);
    write_data(d);
  }*/
}


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

uint32_t cursor = 0;
byte mode = NORMAL;
char SER_BUF[16];

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
  PORTB &= ~0b11;
  NOP; NOP; // just to make sure
  
  uint8_t d = read_data();
  
  PORTB |= 0b11;
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
  PORTB &= ~0b101;
  write_data(data);
  PORTB |= 0b101;
}

void set_addr(uint32_t addr) {
  //ext.writeGPIOAB((uint16_t)addr);
  SPI.transfer16(addr);
  PORTC &= ~0b100; // ADL low

  // set ADL high and set a16 correctly in the same write operation
  byte pc = PORTC;
  pc |= 0b100;
  // shift bit 16 of addr into bit 3, hence shift of 13
  pc |= (addr >> 13) & 0b1000;
  //Serial.println(addr >> 13, BIN);
  //Serial.println(pc, BIN);
  PORTC = pc;
}

// DATA BUS is port c [0,1] and port d [2, 7]

void set_dout_mode() {
  DDRC |= 0b11; // me when i twiddle bits
  DDRD |= ~0b11;
}
// SAFETY: set_dout_mode first
void write_data(uint8_t data) {
  uint8_t low = data & 0b11;
  uint8_t high = data & ~0b11;

  PORTC = (PORTC & ~0b11) | low; // the joy of rmw
  PORTD = (PORTD & 0b11) | high;
}

void set_din_mode() {
  DDRC &= ~0b11;
  DDRD &= 0b11;
}
// SAFETY: set_din_mode first
uint8_t read_data() {
  uint8_t low = PINC & 0b11;
  uint8_t high = PIND & ~0b11;
  return high | low;
}

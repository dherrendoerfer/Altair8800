// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation,
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
// -----------------------------------------------------------------------------

#ifdef __MK66FX1M0__

#include <Arduino.h>
//#include <DueFlashStorage.h>
#include "Altair8800.h"
#include "config.h"
#include "host_teensy36.h"
#include "mem.h"
#include "cpucore.h"
#include "serial.h"
#include "timer.h"
#include "dazzler.h"
//#include "soft_uart.h"

#include <SPI.h>
#include <SdFat.h>

static SdFatSdio SD;

#define min(x, y) ((x)<(y) ? (x) : (y))
#define max(x, y) ((x)>(y) ? (x) : (y))

#include <WS2812Serial.h>

IntervalTimer updateTimer;

volatile uint16_t kbd_state = 10;
volatile uint8_t switch_bank0,switch_bank1,switch_bank2,switch_bank3;
volatile uint8_t state_bank0,state_bank1,state_bank2,state_bank3;
volatile uint8_t rising_edge_bank2,rising_edge_bank3;
volatile boolean switch_change;

const int numled = 42;
const int pin = 10;

#define LEDCOLOR 0x444444
#define LEDOFF   0x000000

byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED
WS2812Serial leds(numled, displayMemory, drawingMemory, pin, WS2812_GRB);

volatile uint16_t addr_led_local;
volatile uint16_t data_led_local;
volatile uint32_t status_led_local;


/*
  NOTE:
  Change -Os to -O3 (to switch optimization from size to performance) in:
  c:\Users\[user]\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.9\platform.txt

  ---- front panel connections by function:

  For pins that are not labeled on the board with their digital number
  the board label is given in []

  Function switches:
     RUN          => DATA7 @ ROW3
     STOP         => DATA7 @ ROW4
     STEP         => DATA6 @ ROW4
     SLOW         => DATA6 @ ROW3
     EXAMINE      => DATA5 @ ROW4
     EXAMINE NEXT => DATA5 @ ROW4
     DEPOSIT      => DATA4 @ ROW4
     DEPOSIT NEXT => DATA4 @ ROW3
     RESET        => DATA3 @ ROW4
     CLR          => DATA3 @ ROW3
     PROTECT      => DATA2 @ ROW4
     UNPROTECT    => DATA2 @ ROW3
     AUX1 UP      => DATA1 @ ROW4
     AUX1 DOWN    => DATA1 @ ROW3
     AUX2 UP      => DATA0 @ ROW4
     AUX2 DOWN    => DATA3 @ ROW3

   Address switches:
     SW0...7      => DATA & ROW1
     SW8...15     => DATA @ ROW2

   Bus LEDs:
     A0..7        => 21 - 28
     A8..15       => 30, 32 - 37, 39
     D0..8        => 20 - 13

   Status LEDs:
     INT          => 9
     WO           => 8
     STACK        => 7
     HLTA         => 6
     OUT          => 5
     M1           => 4
     INP          => 3
     MEMR         => 2
     PROT         => 1
     INTE         => 0
     WAIT         => 41
     HLDA         => 40


  ---- front panel connections by Teensy pin:

  DATA is GPIO Port B: (not all in bit order)
    0  => 16 (in)
    1  => 17 (in)
    2  => 19 (in)
    3  => 18 (in)
    16  => 0 (in)
    17  => 1 (in)
    18  => 31 (in)
    19  => 32 (in)

   ROW selects are:
    0 (ROW1) => 33 (out)
    1 (ROW2) => 24 (out)
    2 (ROW3) => 3 (out)
    3 (ROW4) => 4 (out)

*/


//------------------------------------------------------------------------------------------------------


uint16_t host_read_addr_switches()
{
  return (switch_bank1 << 16) | switch_bank0;
}


//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------


class HLDAGuard
{
public:
  HLDAGuard()  { m_hlda = (host_read_status_leds() & ST_HLDA)!=0; }
  ~HLDAGuard() { if( m_hlda ) host_set_status_led_HLDA(); else host_clr_status_led_HLDA(); }

private:
  bool m_hlda;
};


// The Due has 512k FLASH memory (addresses 0x00000-0x7ffff).
// We use 16k (0x4000 bytes) for storage
// DueFlashStorage address 0 is the first address of the second memory bank,
// i.e. 0x40000. We add 0x3C000 so we use at 0x7C000-0x7ffff
// => MUST make sure that our total program size (shown in Arduine IDE after compiling)
//    is less than 507903 (0x7Bfff)! Otherwise we would overwrite our own program when
//    saving memory pages.
//#define FLASH_STORAGE_OFFSET 0x3C000
//DueFlashStorage dueFlashStorage;
uint32_t due_storagesize = 0x4000;

#define MOVE_BUFFER_SIZE 1024
byte moveBuffer[MOVE_BUFFER_SIZE];

static bool use_sd = false;
static File storagefile;

bool host_storage_init(bool write)
{
  host_storage_close();

  storagefile = SD.open("STORAGE.DAT", write ? FILE_WRITE : FILE_READ);
  if( storagefile )
    {
      // when using the storage file we can provide more memory than with FLASH
      due_storagesize = 512*1024;
      return true;
    }
  else
    return false;
}


void host_storage_close()
{
  if( storagefile ) storagefile.close();
}


void host_storage_invalidate()
{
  host_storage_close();
  SD.remove("STORAGE.BAK");
  SD.rename("STORAGE.DAT", "STORAGE.BAK");
}


static void host_storage_write_flash(const void *data, uint32_t addr, uint32_t len)
{
  uint32_t offset = addr & 3;
  if( offset != 0)
    {
      byte buf[4];
      uint32_t alignedAddr = addr & 0xfffffffc;
//      memcpy(buf, dueFlashStorage.readAddress(FLASH_STORAGE_OFFSET + alignedAddr), 4);
      memcpy(buf+offset, data, min(4-offset, len));
//      dueFlashStorage.write(FLASH_STORAGE_OFFSET + alignedAddr, buf, 4);
//      if( offset + len > 4 )
//        dueFlashStorage.write(FLASH_STORAGE_OFFSET + alignedAddr + 4, ((byte *) data) + (4-offset), len - (4-offset));
    }
//  else
//    dueFlashStorage.write(FLASH_STORAGE_OFFSET + addr, (byte *) data, len);
}


static void host_storage_read_flash(void *data, uint32_t addr, uint32_t len)
{
//  memcpy(data, dueFlashStorage.readAddress(FLASH_STORAGE_OFFSET + addr), len);
}


static void host_storage_write_sd(const void *data, uint32_t addr, uint32_t len)
{
  HLDAGuard hlda;
  if( host_filesys_file_seek(storagefile, addr) )
    {
      storagefile.write((byte *) data, len);
      storagefile.flush();
    }
}


static void host_storage_read_sd(void *data, uint32_t addr, uint32_t len)
{
  HLDAGuard hlda;
  if( storagefile.seek(addr) )
    storagefile.read((byte *) data, len);
}


void host_storage_write(const void *data, uint32_t addr, uint32_t len)
{
  if( storagefile )
    host_storage_write_sd(data, addr, len);
  else
    host_storage_write_flash(data, addr, len);
}

void host_storage_read(void *data, uint32_t addr, uint32_t len)
{
  if( storagefile )
    host_storage_read_sd(data, addr, len);
  else
    host_storage_read_flash(data, addr, len);
}


void host_storage_move(uint32_t to, uint32_t from, uint32_t len)
{
  uint32_t i;
  if( from < to )
    {
      for(i=0; i+MOVE_BUFFER_SIZE<len; i+=MOVE_BUFFER_SIZE)
        {
          host_storage_read(moveBuffer, from+len-i-MOVE_BUFFER_SIZE, MOVE_BUFFER_SIZE);
          host_storage_write(moveBuffer, to+len-i-MOVE_BUFFER_SIZE, MOVE_BUFFER_SIZE);
        }

      if( i<len )
        {
          host_storage_read(moveBuffer, from, len-i);
          host_storage_write(moveBuffer, to, len-i);
        }
    }
  else
    {
      for(i=0; i+MOVE_BUFFER_SIZE<len; i+=MOVE_BUFFER_SIZE)
        {
          host_storage_read(moveBuffer, from+i, MOVE_BUFFER_SIZE);
          host_storage_write(moveBuffer, to+i, MOVE_BUFFER_SIZE);
        }

      if( i<len )
        {
          host_storage_read(moveBuffer, from+i, len-i);
          host_storage_write(moveBuffer, to+i, len-i);
        }
    }
}


//------------------------------------------------------------------------------------------------------


File host_filesys_file_open(const char *filename, bool write)
{
  HLDAGuard hlda;
  return SD.open(filename, write ? FILE_WRITE : FILE_READ);
}


uint32_t host_filesys_file_read(File &f, uint32_t len, void *buffer)
{
  HLDAGuard hlda;
  return f.read((uint8_t *) buffer, len);
}


uint32_t host_filesys_file_write(File &f, uint32_t len, const void *buffer)
{
  HLDAGuard hlda;
  return f.write((const uint8_t *) buffer, len);
}


uint32_t host_filesys_file_set(File &f, uint32_t len, byte b)
{
  HLDAGuard hlda;
  uint32_t res = 0;

  // write data in MOVE_BUFFER_SIZE chunks
  memset(moveBuffer, b, min(len, MOVE_BUFFER_SIZE));
  for(uint32_t i=0; i<len; i+=MOVE_BUFFER_SIZE)
    res += f.write(moveBuffer, min(len-i, MOVE_BUFFER_SIZE));

  return res;
}


void host_filesys_file_flush(File &f)
{
  HLDAGuard hlda;
  f.flush();
}


bool host_filesys_file_seek(File &f, uint32_t pos)
{
  HLDAGuard hlda;

  f.seek(pos);
  if( f.position()<pos && !f.isReadOnly() )
    {
      // if we are seeking past the end of a writable
      // file then expand its size accordingly
      host_filesys_file_set(f, pos-f.position(), 0);
    }

  return f.position()==pos;
}


uint32_t host_filesys_file_pos(File &f)
{
  HLDAGuard hlda;
  return f.position();
}


bool host_filesys_file_eof(File &f)
{
  HLDAGuard hlda;
  return f.isReadOnly() ? f.available()==0 : false;
}


void host_filesys_file_close(File &f)
{
  HLDAGuard hlda;
  f.close();
}


uint32_t host_filesys_file_size(const char *filename)
{
  HLDAGuard hlda;
  int res = -1;

  File f = SD.open(filename, FILE_READ);
  if( f )
    {
      res = f.size();
      f.close();
    }

  return res;
}


bool host_filesys_file_exists(const char *filename)
{
  HLDAGuard hlda;
  return SD.exists(filename);
}


bool host_filesys_file_remove(const char *filename)
{
  HLDAGuard hlda;
  return SD.remove(filename);
}


bool host_filesys_file_rename(const char *from, const char *to)
{
  HLDAGuard hlda;
  return SD.rename(from, to);
}


File host_filesys_dir_open()
{
  HLDAGuard hlda;
  return SD.open("/");
}


const char *host_filesys_dir_nextfile(File &d)
{
  HLDAGuard hlda;
  static char buffer[15];

  while( true )
    {
      File entry = d.openNextFile();
      if( entry )
        {
          if( entry.isFile() )
            {
              entry.getSFN(buffer);
              entry.close();
              return buffer;
            }

          entry.close();
        }
      else
        return NULL;
    }
}


void host_filesys_dir_rewind(File &d)
{
  HLDAGuard hlda;
  d.rewindDirectory();
}


void host_filesys_dir_close(File &d)
{
  HLDAGuard hlda;
  d.close();
}


bool host_filesys_ok()
{
  return use_sd;
}


//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------





void host_serial_setup(byte iface, uint32_t baud, uint32_t config, bool set_primary_interface)
{
  if( iface==1 )
    {
      Serial1.begin(baud, config);
      Serial1.setTimeout(10000);
    }
}


void host_serial_end(byte i)
{
  switch( i )
    {
    case 1: Serial1.end(); break;
    }
}

int host_serial__available(byte i)
{
  switch( i )
    {
    case 0: return Serial.available(); break;
    case 1: return Serial1.available(); break;
    }

 return 0;
}

int host_serial__available_for_write(byte i)
{
  switch( i )
    {
    case 0: return Serial.availableForWrite(); break;
    case 1: return Serial1.availableForWrite(); break;
    }

 return 0;
}

int host_serial_peek(byte i)
{
  switch( i )
    {
    case 0: return Serial.peek(); break;
    case 1: return Serial1.peek(); break;
    }

 return -1;
}

int host_serial_read(byte i)
{
  switch( i )
    {
    case 0: return Serial.read(); break;
    case 1: return Serial1.read(); break;
    }

  return -1;
}

void host_serial_flush(byte i)
{
  switch( i )
    {
    case 0: Serial.flush(); break;
    case 1: Serial1.flush(); break;
    }
}

size_t host_serial_write(byte i, uint8_t b)
{
  switch( i )
    {
    case 0: return Serial.write(b); break;
    case 1: return Serial1.write(b); break;
    }

  return 0;
}

size_t host_serial_write(byte i, const char *buf, size_t n)
{
  size_t a;
  switch( i )
    {
    case 0:
      {
        if( Serial.availableForWrite()<n ) 
          n = Serial.availableForWrite();
        for(a=0; a<n; a++) 
          Serial.write(buf[a]);
        return a;
      }

    case 1:
      {
        if( Serial1.availableForWrite()<n ) 
          n = Serial1.availableForWrite();
        for(a=0; a<n; a++) 
          Serial1.write(buf[a]);
        return a;
      }
    }

  return 0;
}


bool host_serial_ok(byte i)
{
  switch( i )
    {
    case 0: return (bool) Serial; break;
    case 1: return (bool) Serial1; break;
    }

  return false;
  }


const char *host_serial_port_name(byte i)
{
  switch(i)
    {
    case 0: return "USB Programming Port";
    case 1: return "Serial1 (pin 26/27)";
    default: return "???";
    }
}


bool host_serial_port_baud_limits(byte i, uint32_t *min, uint32_t *max)
{
  switch(i)
    {
    case 0: *min = 600;    *max = 1050000; break;
    case 1: *min = 110;    *max = 1050000; break;
    default: return false;
    }

  return true;
}


bool host_serial_port_has_configs(byte i)
{
  return i==1 || i==3 || i==4;
}

void host_check_interrupts() 
{ 
  if( Serial.available() ) 
    serial_receive_host_data(0, Serial.read());

  if( Serial1.available() ) 
    serial_receive_host_data(1, Serial1.read());

}

void host_serial_interrupts_pause()
{
  return;
}
void host_serial_interrupts_resume()
{
  return;
}


// --------------------------------------------------------------------------------------------------


static const uint16_t function_switch_id[16] = {0x80,0x8000,0x4000,0x40,0x2000,0x20,0x1000,0x10,0x800,0x8,0x400,0x4,0x200,0x2,0x100,0x1};

uint8_t function_switch_bank2_irq[16] = { INT_SW_AUX2DOWN>>24, 0, 0, INT_SW_CLR>>24, 0, 0, 0, 0 };
uint8_t function_switch_bank3_irq[16] = { INT_SW_AUX2UP>>24, 0, 0, INT_SW_RESET>>24, 0, 0, 0, INT_SW_STOP>>24};

bool host_read_function_switch(byte i)
{
  int index = function_switch_id[i];
  if (index > 0xff)
    return (switch_bank3 & (index>>8));

  return (switch_bank2 & (index));
}

void do_altair_interrupt_bank2(byte i)
{
  if (function_switch_bank2_irq[i] != 0)
    altair_interrupt(function_switch_bank2_irq[i]<<24);
}

void do_altair_interrupt_bank3(byte i)
{
  if (function_switch_bank3_irq[i] != 0)
    altair_interrupt(function_switch_bank3_irq[i]<<24);
}

bool host_read_function_switch_debounced(byte i)
{
  return host_read_function_switch(i);
}


bool host_read_function_switch_edge(byte i)
{
  //
  // Note: reading an edge switch event also clears the bit
  //
  int index = function_switch_id[i];
  boolean ret;
  if (index > 0xff){
    ret = (rising_edge_bank3 & (index>>8));
    if (ret)
      rising_edge_bank3 &= ~(index>>8);
    return ret;
  }

  ret = (rising_edge_bank2 & (index));
  if (ret)
    rising_edge_bank2 &= ~(index);
  return ret;
}


uint16_t host_read_function_switches_edge()
{
  uint16_t res = 0;

  for (int i=0 ; i<16 ; i++) {
    if (host_read_function_switch_edge(i))
      res |= 1<<i;
  }
  return res;
}


void host_reset_function_switch_state()
{
  switch_change = false;
  rising_edge_bank2 = 0;
  rising_edge_bank3 = 0;
}

uint8_t host_read_sense_switches()
{
  return switch_bank1;
}


// --------------------------------------------------------


void host_copy_flash_to_ram(void *dst, const void *src, uint32_t len)
{
  memcpy(dst, src, len);
}


uint32_t host_get_random()
{
  delayMicroseconds(1);
  return (((uint32_t) random(0,65535)) * 65536l | random(0,65535));
}


bool host_is_reset()
{
  return host_read_function_switch(SW_RESET);
}


#include <malloc.h>
 char _end;
extern "C" char *sbrk(int i);
char *ramstart=(char *)0x20070000;
char *ramend=(char *)0x20088000;

void host_system_info()
{
  Serial.println("Host is Teensy\n");

  struct mallinfo mi=mallinfo();
  char *heapend=sbrk(0);
  register char * stack_ptr asm("sp");

  Serial.print("RAM Start        : 0x"); Serial.println((unsigned long)ramstart, HEX);
  Serial.print("Data/Bss end     : 0x"); Serial.println((unsigned long)&_end, HEX);
  Serial.print("Heap End         : 0x"); Serial.println((unsigned long)heapend, HEX);
  Serial.print("Stack Pointer    : 0x"); Serial.println((unsigned long)stack_ptr, HEX);
  Serial.print("RAM End          : 0x"); Serial.println((unsigned long)ramend, HEX);
  Serial.print("Heap RAM Used    : "); Serial.println(mi.uordblks);
  Serial.print("Program RAM Used : "); Serial.println(&_end - ramstart);
  Serial.print("Stack RAM Used   : "); Serial.println(ramend - stack_ptr);
  Serial.print("Free RAM         : "); Serial.println(stack_ptr - heapend + mi.fordblks);
  Serial.print("Data storage     : ");

#if USE_HOST_FILESYS>0
  Serial.print("SD card file system");
  if( SD.card()->errorCode()==SD_CARD_ERROR_NONE )
    { Serial.print(" ("); Serial.print(SD.card()->cardSize() / (2*1024)); Serial.println("M)"); }
  else
    { Serial.print(" (card error: 0x"); Serial.print(SD.card()->errorCode(), HEX); Serial.println(")"); }
#else
  Serial.print(storagefile ? "STORAGE.DAT file on SD card" : "flash memory");
  Serial.print(" ("); Serial.print(due_storagesize / 1024); Serial.print("K)");
#if NUM_DRIVES>0 || NUM_HDSK_UNITS>0
  if( SD.card()->errorCode()!=SD_CARD_ERROR_NONE )
    { Serial.print(" (SD card error: 0x"); Serial.print(SD.card()->errorCode(), HEX); Serial.print(")"); }
  Serial.println();
#endif
#endif
}


// -----------------------------------------------------------------------------


void panelUpdate()
{
  uint32_t tmp;
  switch(kbd_state) {
    case 10: //Setup: Set all ROWs high
      digitalWriteFast(33,HIGH);
      digitalWriteFast(24,HIGH);
      digitalWriteFast(3,HIGH);
      digitalWriteFast(4,HIGH);
      kbd_state=0;
      break;    
    case 0: //ROW 1 LOW
      digitalWriteFast(4,HIGH); digitalWriteFast(33,LOW);
      break;
    case 1: //READ Switches 0-7
      tmp=~GPIOB_PDIR;
      switch_bank0 = ((tmp & 0x030000)>>12) | ((tmp & 0x0C00)>>4) | (tmp & 0x0f);
      if (state_bank0 != switch_bank0) {
        state_bank0 = switch_bank0;
        switch_change = true;  
      }
      break;
    case 2: //ROW 2 LOW
      digitalWriteFast(33,HIGH); digitalWriteFast(24,LOW);
      break;
    case 3: //READ Switches 8-15
      tmp=~GPIOB_PDIR;
      switch_bank1 = ((tmp & 0x030000)>>12) | ((tmp & 0x0C00)>>4) | (tmp & 0x0f);
      if (state_bank1 != switch_bank1) {
        state_bank1 = switch_bank1;
        switch_change = true;  
      }
      break;
    case 4: //ROW 2 LOW
      digitalWriteFast(24,HIGH); digitalWriteFast(3,LOW);
      break;
    case 5: //READ Function Switches 0-7 Down position
      tmp=~GPIOB_PDIR;
      switch_bank2 = ((tmp & 0x030000)>>12) | ((tmp & 0x0C00)>>4) | (tmp & 0x0f);
      rising_edge_bank2 = 0; 
      if (state_bank2 != switch_bank2) {
        for (int i=0;i<8;i++) { 
          if((switch_bank2 & (1<<i)) != (state_bank2 & (1<<i))){
            if (switch_bank2 & (1<<i)) {
              rising_edge_bank2 += 1<<i;
              do_altair_interrupt_bank2(i);
            }
          }
        }
        state_bank2 = switch_bank2;
        switch_change = true;  
      }

      break;
    case 6: //ROW 2 LOW
      digitalWriteFast(3,HIGH); digitalWriteFast(4,LOW);
      break;
    case 7: //READ Function Switches 0-7 Down position
      tmp=~GPIOB_PDIR;
      switch_bank3 = ((tmp & 0x030000)>>12) | ((tmp & 0x0C00)>>4) | (tmp & 0x0f);
      rising_edge_bank3 = 0;
      if (state_bank3 != switch_bank3) {
        for (int i=0;i<8;i++) { 
          if((switch_bank3 & (1<<i)) != (state_bank3 & (1<<i))) {
            if (switch_bank3 & (1<<i)) {
              rising_edge_bank3 += 1<<i;
              do_altair_interrupt_bank3(i);
            }
          }
        }
        state_bank3 = switch_bank3;
        switch_change = true;  
      }
      break;
  }  
  kbd_state++;
  if (kbd_state == 8) {
    kbd_state=0;
    updateStatusLEDs();
  }
    leds.show();
}


void updateAddressLEDs()
{
  for (int i=0;i<9;i++) {
    if (addr_led_local & (1<<i))
      leds.setPixel(21+i,LEDCOLOR);
    else
      leds.setPixel(21+i,LEDOFF);
  }
  for (int i=9;i<15;i++) {
    if (addr_led_local & (1<<i))
      leds.setPixel(22+i,LEDCOLOR);
    else
      leds.setPixel(22+i,LEDOFF);
  }
  if (addr_led_local & (1<<15))
    leds.setPixel(22+16,LEDCOLOR);
  else
    leds.setPixel(22+16,LEDOFF);

}

void updateDataLEDs()
{
  for (int i=0;i<8;i++) {
    if (data_led_local & (1<<i))
      leds.setPixel(20-i,LEDCOLOR);
    else
      leds.setPixel(20-i,LEDOFF);
  }
}

void updateStatusLEDs()
{
  for (int i=0;i<9;i++) {
    if (status_led_local & (1<<i))
      leds.setPixel(9-i,LEDCOLOR);
    else
      leds.setPixel(9-i,LEDOFF);
  }

  if (status_led_local & (1<<10)) //HLDA
      leds.setPixel(40,LEDCOLOR);
  else
      leds.setPixel(40,LEDOFF);

  if (status_led_local & (1<<11)) //WAIT
      leds.setPixel(41,LEDCOLOR);
  else
      leds.setPixel(41,LEDOFF);
}

void host_setup()
{
  //On-board LED
  pinMode(13,OUTPUT);
  
  //Reroute serial1
  pinMode(26,OUTPUT);
  pinMode(27,INPUT_PULLUP);
  Serial1.setTX(26);
  Serial1.setRX(27);

  // Set in/out for pins
  //PORT A, Switch rows
  pinMode(33,OUTPUT);
  pinMode(24,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);

  //Port B, Switch inputs
  pinMode(16,INPUT_PULLUP);
  pinMode(17,INPUT_PULLUP);
  pinMode(19,INPUT_PULLUP);
  pinMode(18,INPUT_PULLUP);
  pinMode(0,INPUT_PULLUP);
  pinMode(1,INPUT_PULLUP);
  pinMode(31,INPUT_PULLUP);
  pinMode(32,INPUT_PULLUP);

  //Analog/Sound out
  pinMode(21,OUTPUT);
  analogWrite(A21,4095);

  //Start update timer
  updateTimer.begin(panelUpdate, 5000);

  //Init non-blocking WS2812b lib
  leds.begin();

#if NUM_DRIVES>0 || NUM_HDSK_UNITS>0 || USE_HOST_FILESYS>0
  // check if SD card available (send "chip select" signal to HLDA status light)
  HLDAGuard hlda;
  // SdInfo.h in the SdFat library says: Set SCK rate to F_CPU/3 (SPI_DIV3_SPEED) for Due
  // (84MHZ/3 = 28MHz). If that fails try 4MHz and if that fails too then fall back to 250Khz.
  // If neither of those work then something is seriously wrong.
  if( SD.begin())
    {
#if USE_HOST_FILESYS>0
      // storing configurations etc directly on SD card
      use_sd = true;
#else
      // not using host file system => open storage (file) for writing
      use_sd = host_storage_init(true);
#endif
    }
#endif

  randomSeed(analogRead(A0));
}


#endif

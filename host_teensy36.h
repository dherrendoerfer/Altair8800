// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// Copyright (C) 2020 Dirk Herrendoerfer
// -----------------------------------------------------------------------------

#ifndef HOST_TEENSY_H
#define HOST_TEENSY_H

#include "config.h"
#include <SdFat.h>

// Teensy provides a file system (via SD card)
#define HOST_HAS_FILESYS
#define HOST_FILESYS_FILE_TYPE File
#define HOST_FILESYS_DIR_TYPE  File

#define MEMSIZE 0x10000

#define HOST_STORAGESIZE 1024*512
#define HOST_BUFFERSIZE  0x100

#define HOST_PERFORMANCE_FACTOR 1.0

#define HOST_NUM_SERIAL_PORTS   (4)


// ------------------------------------------ switches


uint8_t host_read_sense_switches();

uint16_t host_read_addr_switches();


// ------------------------------------------ status LEDs
extern volatile uint32_t status_led_local;


/* reading global variables is faster than reading back the i/o register
   => INTE and WAIT are read often so we keep their state in a global variable */

#define host_set_status_led_INT()     status_led_local |= ST_INT
#define host_set_status_led_WO()      status_led_local |= ST_WO
#define host_set_status_led_STACK()   status_led_local |= ST_STACK
#define host_set_status_led_HLTA()    status_led_local |= ST_HLTA
#define host_set_status_led_OUT()     status_led_local |= ST_OUT
#define host_set_status_led_M1()      status_led_local |= ST_M1
#define host_set_status_led_INP()     status_led_local |= ST_INP
#define host_set_status_led_MEMR()    status_led_local |= ST_MEMR
#define host_set_status_led_INTE()    status_led_local |= ST_INTE;
#define host_set_status_led_PROT()    status_led_local |= ST_PROT
#define host_set_status_led_WAIT()  { status_led_local |= ST_WAIT; status_wait = true; }
#define host_set_status_led_HLDA()    status_led_local |= ST_HLDA

#define host_clr_status_led_INT()     status_led_local &= ~(ST_INT)
#define host_clr_status_led_WO()      status_led_local &= ~(ST_WO)
#define host_clr_status_led_STACK()   status_led_local &= ~(ST_STACK)
#define host_clr_status_led_HLTA()    status_led_local &= ~(ST_HLTA)
#define host_clr_status_led_OUT()     status_led_local &= ~(ST_OUT)
#define host_clr_status_led_M1()      status_led_local &= ~(ST_M1)
#define host_clr_status_led_INP()     status_led_local &= ~(ST_INP)
#define host_clr_status_led_MEMR()    status_led_local &= ~(ST_MEMR)
#define host_clr_status_led_INTE()    status_led_local &= ~(ST_INTE);
#define host_clr_status_led_PROT()    status_led_local &= ~(ST_PROT)
#define host_clr_status_led_WAIT()  { status_led_local &= ~(ST_WAIT); status_wait = false; }
#define host_clr_status_led_HLDA()    status_led_local &= ~(ST_HLDA)

#define host_read_status_led_WAIT()   status_wait
#define host_read_status_led_M1()     status_led_local & (ST_M1)
#define host_read_status_led_HLTA()   status_led_local & (ST_HLTA)
#define host_read_status_led_INTE()   status_inte

// reading from memory (MEMR on, WO on)
#define host_set_status_leds_READMEM()        status_led_local |= (ST_MEMR | ST_WO)

// reading opcode from memory (MEMR on, M1 on, WO on)
#define host_set_status_leds_READMEM_M1()     status_led_local |= (ST_MEMR | ST_M1 | ST_WO)

// reading from stack (MEMR on, WO on, STACK on)
#define host_set_status_leds_READMEM_STACK()  status_led_local |= (ST_MEMR | ST_WO | ST_STACK)

// writing to memory (MEMR off, WO off)
#define host_set_status_leds_WRITEMEM()       status_led_local &= ~(ST_MEMR | ST_WO)

inline uint16_t host_read_status_leds()
{
  return status_led_local;
}


// ----------------------------------------------------- address bus

extern volatile uint16_t addr_led_local;

inline void host_set_addr_leds(uint16_t v)
{
  addr_led_local = v;
}

inline uint16_t host_read_addr_leds()
{
  return addr_led_local;
}


// ---------------------------------------------------- data bus
extern volatile uint16_t data_led_local;

inline uint16_t host_set_data_leds(uint16_t v)
{
  data_led_local = v;
  return v;
}

inline byte host_read_data_leds()
{
  return data_led_local;
}

// ---------------------------------------------------- interrupts

// On the Teensy we are not using real interrupts for serial.
void host_check_interrupts();

void host_serial_interrupts_pause();
void host_serial_interrupts_resume();

#endif

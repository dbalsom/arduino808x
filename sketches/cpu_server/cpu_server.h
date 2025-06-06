/*
    Arduino8088 Copyright 2022-2025 Daniel Balsom
    https://github.com/dbalsom/arduino_8088

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the “Software”),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER   
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/
#ifndef _CPU_SERVER_H
#define _CPU_SERVER_H

// Baud rate is ignored for Arduino DUE as it uses native SerialUSB.
//#define BAUD_RATE 460800
#define BAUD_RATE 1000000

// Debug baud rate controls Serial1 speed. Check the documentation of your RS232 interface
// for maximum rated speed. Exceeding it will cause dropped characters or other corruption.
//#define DEBUG_BAUD_RATE 230400
#define DEBUG_BAUD_RATE 460800

#define CMD_TIMEOUT 100 // Command timeout in milliseconds
#define MAX_COMMAND_BYTES 28 // Maximum length of command parameter input

#define MODE_ASCII 0 // Use ASCII response codes (for interactive debugging only, client won't support)

#define BRKEM_VECTOR ((uint8_t)0x00)

// Print a dot to the debugging output on each load.
#define LOAD_INDICATOR 1
#define STORE_INDICATOR 1

#define TRACE_ALL 0

// These defines control tracing and debugging output for each state.
// Note: tracing a STORE operation will likely cause it to timeout on the client.
#define TRACE_RESET (0 | TRACE_ALL)
#define TRACE_VECTOR (0 | TRACE_ALL)
#define TRACE_LOAD (1 | TRACE_ALL)
#define TRACE_ID (0 | TRACE_ALL)
#define TRACE_EMU_ENTER (1 | TRACE_ALL)
#define TRACE_EMU_EXIT (1 | TRACE_ALL)
#define TRACE_EXECUTE (1 | TRACE_ALL)
#define TRACE_STORE (1 | TRACE_ALL)
#define TRACE_FINALIZE (1 | TRACE_ALL)
// Debugging output for queue operations (flushes, regular queue ops are always reported)
#define TRACE_QUEUE (0 | TRACE_ALL)

#define DEBUG_ALL 0

// Report state changes and time spent in each state
#define DEBUG_STATE (1 | DEBUG_ALL)
#define DEBUG_RESET (0 | DEBUG_ALL)
#define DEBUG_LOAD_DONE (1 | DEBUG_ALL)
#define DEBUG_STORE (1 | DEBUG_ALL)
#define DEBUG_FINALIZE (0 | DEBUG_ALL)
#define DEBUG_INSTR (0 | DEBUG_ALL) // Print instruction mnemonics as they are executed from queue
#define DEBUG_EMU (1 | DEBUG_ALL) // Print debugging information concerning 8080 emulation mode state
#define DEBUG_LOCK (0 | DEBUG_ALL) // Print a message when the LOCK pin is asserted on a cycle

#define DEBUG_PROTO 0 // Insert debugging messages into serial output (Escaped by ##...##)
#define DEBUG_CMD 0

#define MAX_ERR_LEN 50 // Maximum length of an error string


#define FINALIZE_TIMEOUT 30
#define FINALIZE_EMU_TIMEOUT 90 // We need more time to exit emulation mode
#define STORE_TIMEOUT 300


const char RESPONSE_CHRS[] = {
  '!', '.'
};

const char VERSION_DAT[] = {
  'a', 'r', 'd', '8', '0', '8', '8'
};

const uint8_t VERSION_NUM = 2;

typedef enum {
  CmdNone            = 0x00,
  CmdVersion         = 0x01,
  CmdReset           = 0x02,
  CmdLoad            = 0x03,
  CmdCycle           = 0x04,
  CmdReadAddressLatch= 0x05,
  CmdReadStatus      = 0x06,
  CmdRead8288Command = 0x07,
  CmdRead8288Control = 0x08, 
  CmdReadDataBus     = 0x09,
  CmdWriteDataBus    = 0x0A,
  CmdFinalize        = 0x0B,
  CmdBeginStore      = 0x0C,
  CmdStore           = 0x0D,
  CmdQueueLen        = 0x0E,
  CmdQueueBytes      = 0x0F,
  CmdWritePin        = 0x10,
  CmdReadPin         = 0x11,
  CmdGetProgramState = 0x12,
  CmdLastError       = 0x13,
  CmdGetCycleState   = 0x14,
  CmdCycleGetCycleState = 0x15,
  CmdPrefetchStore   = 0x16,
  CmdReadAddress     = 0x17,
  CmdCpuType         = 0x18,
  CmdEmulate8080     = 0x19,
  CmdInvalid         = 0x1A,
} server_command;

const char *CMD_STRINGS[] = {
  "NONE",
  "VERSION",
  "RESET",
  "LOAD",
  "CYCLE",
  "READADDRLATCH",
  "READSTATUS",
  "READ8288CMD",
  "READ8288CTRL",
  "READDATABUS",
  "WRITEDATABUS",
  "FINALIZE",
  "BEGINSTORE",
  "STORE",
  "QUEUELEN",
  "QUEUEBYTES",
  "WRITEPIN",
  "READPIN",
  "GETPGMSTATE",
  "GETLASTERR",
  "GETCYCLESTATE",
  "CGETCYCLESTATE",
  "PREFETCHSTORE",
  "READADDRBUS",
  "CPUTYPE",
  "EMULATE8080",
  "INVALID",
};

typedef bool (*command_func)();

#define RESPONSE_FAIL 0x00
#define RESPONSE_OK 0x01

// ASCII aliases for commands, mostly for interactive debugging
const uint8_t CMD_ALIASES[] = {
  0, // CmdNone
  'v', // CmdVersion
  'r', // CmdReset
  'l', // CmdLoad
  'c', // CmdCycle
  'a', // CmdReadAddressLatch
  's', // CmdReadStatus
  't', // CmdRead8288Command
  'u', // CmdRead8288Control
  'r', // CmdReadDataBus
  'w', // CmdWriteDataBus,
  'z', // CmdFinalize
  'm', // CmdBeginStore,
  'w', // CmdStore,
  'q', // CmdQueueLen,
  'b', // CmdQueueBytes,
  'x', // CmdWritePin,
  'y', // CmdReadPin,
  'g', // CmdGetProgramState
  'e', // CmdGetLastError
  'f', // CmdGetCycleStatus
  'k', // CmdPrefetchStore
  'i', // CmdReadAddress
  'd', // CmdCpuType
  'h', // CmdEmulate8080
  0 // CmdInvalid
};

// List of valid arguments to CmdWritePin. Only these specific pins
// can have state written to.
const uint8_t WRITE_PINS[] = {
  6,  // READY
  7,  // TEST
  12, // INTR
  13, // NMI
};

// Number of argument bytes expected for each command
const uint8_t CMD_INPUTS[] = {
  0,  // CmdNone
  0,  // CmdVersion
  0,  // CmdReset
  28, // CmdLoad
  0,  // CmdCycle
  0,  // CmdReadAddressLatch
  0,  // CmdReadStatus
  0,  // CmdRead8288Command 
  0,  // CmdRead8288Control 
  0,  // CmdReadDataBus 
  2,  // CmdWriteDataBus
  0,  // CmdFinalize
  0,  // CmdBeginStore,
  0,  // CmdStore,
  0,  // CmdQueueLen,
  0,  // CmdQueueBytes,
  2,  // CmdWritePin,
  1,  // CmdReadPin,
  0,  // CmdGetProgramState,
  0,  // CmdGetLastError,
  0,  // CmdGetCycleState,
  0,  // CmdCycleGetCycleState,
  0,  // CmdPrefetchStore,
  0,  // CmdReadAddress
  0,  // CmdCpuType
  0,  // CmdEmulate8080
  0,  // CmdInvalid
};

typedef enum {
  WaitingForCommand = 0x01,
  ReadingCommand,
  ExecutingCommand
} command_state_t;

typedef struct server_state {
  command_state_t c_state;
  server_command cmd;
  uint8_t cmd_byte_n;
  uint8_t cmd_bytes_expected;
  uint32_t cmd_start_time;
} Server;

bool cmd_version(void);
bool cmd_reset(void);
bool cmd_load(void);
bool cmd_cycle(void);
bool cmd_read_address_latch(void);
bool cmd_read_status(void);
bool cmd_read_8288_command(void);
bool cmd_read_8288_control(void);
bool cmd_read_data_bus(void);
bool cmd_write_data_bus(void);
bool cmd_finalize(void);
bool cmd_begin_store(void);
bool cmd_store(void);
bool cmd_queue_len(void);
bool cmd_queue_bytes(void);
bool cmd_write_pin(void);
bool cmd_read_pin(void);
bool cmd_get_program_state(void);
bool cmd_get_last_error(void);
bool cmd_get_cycle_state(void);
bool cmd_cycle_get_cycle_state(void);
bool cmd_prefetch_store(void);
bool cmd_read_address(void);
bool cmd_cpu_type(void);
bool cmd_invalid(void);
bool cmd_emu8080(void);

#endif

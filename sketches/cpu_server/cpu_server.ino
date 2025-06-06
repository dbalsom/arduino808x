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
#include <Arduino.h>
#include "arduino8088.h"
#include "cpu_server.h"
#include "opcodes.h"

static Server SERVER;
static Cpu CPU;
static Intel8288 I8288;

registers LOAD_REGISTERS = {
  0x0000, // AX
  0x0000, // BX
  0x0000, // CX
  0x0000, // DX
  0x0000, // SS
  0x0000, // SP
  0x0000, // FLAGS
  0x0000, // IP
  0xFFFF, // CS
  0x0000, // DS
  0x0000, // ES
  0x0000, // BP
  0x0000, // SI
  0x0000, // DI
};

// Register load routine. This program gets patched with the client supplied register values.
// It uses MOVs and POPs to set the register state as specified before the main program execution
// begins.
uint8_t LOAD_PROGRAM[] = {
  0x00, 0x00, 0xB8, 0x00, 0x00, 0x8E, 0xD0, 0x89, 0xC4, 0x9D, 0xBB, 0x00, 0x00, 0xB9, 0x00, 0x00,
  0xBA, 0x00, 0x00, 0xB8, 0x00, 0x00, 0x8E, 0xD0, 0xB8, 0x00, 0x00, 0x8E, 0xD8, 0xB8, 0x00, 0x00,
  0x8E, 0xC0, 0xB8, 0x00, 0x00, 0x89, 0xC4, 0xB8, 0x00, 0x00, 0x89, 0xC5, 0xB8, 0x00, 0x00, 0x89,
  0xC6, 0xB8, 0x00, 0x00, 0x89, 0xC7, 0xB8, 0x00, 0x00, 0xEA, 0x00, 0x00, 0x00, 0x00
};

// Patch offsets for load program
const size_t LOAD_BX = 0x0B;
const size_t LOAD_CX = 0x0E;
const size_t LOAD_DX = 0x11;
const size_t LOAD_SS = 0x14;
const size_t LOAD_DS = 0x19;
const size_t LOAD_ES = 0x1E;
const size_t LOAD_SP = 0x23;
const size_t LOAD_BP = 0x28;
const size_t LOAD_SI = 0x2D;
const size_t LOAD_DI = 0x32;
const size_t LOAD_AX = 0x37;
const size_t LOAD_IP = 0x3A;
const size_t LOAD_CS = 0x3C;

// CPU ID program. This is pretty simple - Intel CPUs have the undocumented and very fast instruction
// SALC at D6 - NEC CPUs have an undefined alias for XLAT that takes a lot longer. We can simply
// measure the execution time to determine Intel vs NEC.
// This routine is run first, in the reset vector, before the Jump program.

// Currently, this program must be able to be fetched in a single m-cycle, so it is limited to one
// opcode and an optional NOP for 16-bit CPUs.
const uint8_t CPUID_PROGRAM[] = {
  0xD6, 0x90
};

// 8080 Emulation enter program. This program executes the BRKEM opcode to enter 8080 emulation 
// on a compatible NEC CPU such as the V20 or V30.
// The first four bytes are used as the BRKEM vector segment and offset, and are patched with the 
// values of CS and IP.
uint8_t EMU_ENTER_PROGRAM[] = {
  0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, BRKEM_VECTOR
};

// 8080 Emulation exit program. This program executes PUSH PSW to preseve the 8080 flag state,
// then POP PSW to restore BP, then executes RETEM to exit emulation mode.
const uint8_t EMU_EXIT_PROGRAM[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 6 NOPs to hide program from client
  0xF5, 0x00, // PUSH PSW, NOP
  0x33, 0x33, // INX SP, INX SP to restore 8080 stack pointer
  0xED, 0xFD, // RETEM
};

// Far Jump program. We feed this program to the CPU at the reset vector. On an 8088 the reset 
// vector is at FFFF:0000 or address FFFF0 - giving us only 16 bytes to the end of the address
// space, where we will wrap around. We could wrap, but it gets a bit confusing, so instead 
// we'll jump to a clean new segment. The exact segment is configurable with LOAD_SEG which
// will get patched into this routine as the destination segment.
uint8_t JUMP_VECTOR[] = {
  0xEA, 0x00, 0x00, 0x00, 0x00
};

// Register store routine.
// Six NOPs have been padded to the front of the STORE routine to hide it from appearing in cycle
// traces.
// We can probably think of a better way to hide STORE program bytes, as this adds several cycles.
const uint8_t STORE_PROGRAM[] = {
  0x90, 0x90, 0x90, 0x90, 0x90, 0x90,
  0xE7, 0xFE, 0x89, 0xD8, 0xE7, 0xFE, 0x89, 0xC8, 0xE7, 0xFE, 0x89, 0xD0, 0xE7, 0xFE, 0x8C, 0xD0,
  0xE7, 0xFE, 0x89, 0xE0, 0xE7, 0xFE, 0xB8, 0x00, 0x00, 0x8E, 0xD0, 0xB8, 0x04, 0x00, 0x89, 0xC4,
  0x9C, 0xE8, 0x00, 0x00, 0x8C, 0xC8, 0xE7, 0xFE, 0x8C, 0xD8, 0xE7, 0xFE, 0x8C, 0xC0, 0xE7, 0xFE,
  0x89, 0xE8, 0xE7, 0xFE, 0x89, 0xF0, 0xE7, 0xFE, 0x89, 0xF8, 0xE7, 0xFE, 0xB0, 0xFF, 0xE6, 0xFD
};

uint8_t COMMAND_BUFFER[MAX_COMMAND_BYTES] = {0};

command_func V_TABLE[] = {
  &cmd_version,
  &cmd_reset,
  &cmd_load,
  &cmd_cycle,
  &cmd_read_address_latch,
  &cmd_read_status,
  &cmd_read_8288_command,
  &cmd_read_8288_control,
  &cmd_read_data_bus,
  &cmd_write_data_bus,
  &cmd_finalize,
  &cmd_begin_store,
  &cmd_store,
  &cmd_queue_len,
  &cmd_queue_bytes,
  &cmd_write_pin,
  &cmd_read_pin,
  &cmd_get_program_state,
  &cmd_get_last_error,
  &cmd_get_cycle_state,
  &cmd_cycle_get_cycle_state,
  &cmd_prefetch_store,
  &cmd_read_address,
  &cmd_cpu_type,
  &cmd_emu8080,
  &cmd_invalid
};

const size_t MAX_CMD = (sizeof V_TABLE / sizeof (command_func));

char LAST_ERR[MAX_ERR_LEN] = {0};

// Main Sketch setup routine
void setup() {
  SERIAL.begin(BAUD_RATE);
  Serial1.begin(DEBUG_BAUD_RATE);
  while (!SERIAL);
  while (!Serial1);
  
  // Wait for CPU to initialize
  delayMicroseconds(200);
  Serial1.write("Init\r\n", 6);  // More explicit raw write

  // Set all output pins to OUTPUT
  for( int p = 0; p < (sizeof OUTPUT_PINS / sizeof OUTPUT_PINS[0]); p++ ) {
    pinMode(OUTPUT_PINS[p], OUTPUT);
  }
  // Set all input pins to INPUT
  for( int p = 0; p < (sizeof INPUT_PINS / sizeof INPUT_PINS[0]); p++ ) {
    pinMode(INPUT_PINS[p], INPUT);
  }

  // Default output pin states
  digitalWrite(RQ_PIN, HIGH); // Don't allow other bus masters
  digitalWrite(READY_PIN, HIGH);
  digitalWrite(TEST_PIN, LOW);
  digitalWrite(INTR_PIN, LOW); // Must set these to a known value or risk spurious interrupts!
  digitalWrite(NMI_PIN, LOW);  // Must set these to a known value or risk spurious interrupts!
  digitalWrite(AEN_PIN, LOW); // AEN is enable-low
  digitalWrite(CEN_PIN, HIGH); // Command enable enables the outputs on the i8288

  // Patch the reset vector jump program
  patch_vector_pgm(JUMP_VECTOR, LOAD_SEG);

  cpu_id();

  CPU.v_state = Reset;

  beep(100);
  Serial1.println("Arduino8088 Server Initialized!");
  Serial1.flush();
  clear_error();
  
  SERVER.c_state = WaitingForCommand;
}

void reset_cpu_struct(bool reset_load_regs) {

  // Retain detected cpu type, width and emulation flags.
  cpu_type_t cpu_type = CPU.cpu_type;
  cpu_width_t width = CPU.width;
  bool do_emulation = CPU.do_emulation;
  registers_t load_regs = CPU.load_regs;

  // Zero the CPU struct
  memset(&CPU, 0, sizeof CPU);

  if(!reset_load_regs) {
    // Restore regs
    CPU.load_regs = load_regs;
  }

  // Restore retained values
  CPU.cpu_type = cpu_type;
  CPU.width = width;
  CPU.do_emulation = do_emulation;

  CPU.state_begin_time = 0;
  change_state(Reset);
  CPU.data_bus = 0x00; 
  init_queue();
}

bool cpu_id() {
    if (!cpu_reset()) {
        Serial1.println("cpu_id(): Failed to reset CPU!");
        set_error("Failed to reset CPU!");
        return false;
    }
    change_state(CpuId);
    uint32_t timeout = 0;
    while (CPU.v_state != LoadDone) {
        cycle();
        timeout++;
        if (timeout > 200) {
            Serial1.println("cpu_id(): CPU ID timeout!");
            set_error("CPU ID timeout!");
            return false;
        }
    }

    size_t t_idx = CPU.cpu_type;
    if (t_idx < CPU_TYPE_COUNT) {
      Serial1.print("cpu_id(): Detected CPU: ");
      Serial1.print(CPU_TYPE_STRINGS[CPU.cpu_type]);
      Serial1.println("");
    }
    else {
      Serial1.println("Bad CPU type!");
      return false;
    }

    return true;
}

// Read a byte from the data bus. The half of the bus to read is determined 
// by BHE and A0.
uint8_t data_bus_read_byte() {
  CPU.data_bus = data_bus_read(CPU.data_width);
  if (!READ_BHE_PIN) {
    // High byte is active.
    return (uint8_t)(CPU.data_bus >> 8);
  }
  else {
    // Low byte is active.
    return (uint8_t)CPU.data_bus;
  }
}

void data_bus_set_byte(uint8_t byte) {
    if (!READ_BHE_PIN) {
    // High byte is active.
    CPU.data_bus = ((uint16_t)byte) << 8;
  }
  else {
    // Low byte is active.
    CPU.data_bus = (uint16_t)byte;
  }
}

void clear_error() {
  strncpy(LAST_ERR, "No error", MAX_ERR_LEN - 1);
}

void set_error(const char *msg) {
  strncpy(LAST_ERR, msg, MAX_ERR_LEN - 1);
  Serial1.println("");
  Serial1.println("************ ERROR ************");
  Serial1.println(LAST_ERR);
  Serial1.println("*******************************");
  error_beep();
}

// Send a failure code byte in response to a failed command
void send_fail() {
  #if MODE_ASCII
    SERIAL.write(RESPONSE_CHRS[RESPONSE_FAIL]);
  #else
    SERIAL.write((uint8_t)RESPONSE_FAIL);
  #endif
}

// Send the success code byte in response to a successful command
void send_ok() {
  #if MODE_ASCII
    SERIAL.write(RESPONSE_CHRS[RESPONSE_OK]);
  #else
    SERIAL.write((uint8_t)RESPONSE_OK);
  #endif
}

void debug_proto(const char* msg) {
  #if DEBUG_PROTO
    Serial1.print("## ");
    Serial1.print(msg);
    Serial1.println(" ##");
  #endif
}

void debug_cmd(const char *cmd, const char* msg) {
  #if DEBUG_PROTO
    Serial1.print("## cmd ");
    Serial1.print(cmd);
    Serial1.print(": " );
    Serial1.print(msg);
    Serial1.println(" ##");
  #endif
}

// Server command - Version
// Send server identifier 'ard8088' followed by protocol version number in binary
bool cmd_version() {
  debug_cmd("VERSION", "In cmd");
  SERIAL.write((uint8_t *)VERSION_DAT, sizeof VERSION_DAT);
  SERIAL.write(VERSION_NUM);
  FLUSH;
  delay(10); // let USB complete the transaction
  Serial1.println("Got version query!");
  return true;
}

// Server command - Reset
// Attempt to reset the CPU and report status.
// This will be rarely used by itself as the register state is not set up. The Load 
// command will reset the CPU and set register state.
bool cmd_reset() {
  debug_cmd("RESET", "In cmd");
  bool result;
  snprintf(LAST_ERR, MAX_ERR_LEN, "NO ERROR");

  #if EMULATE_8288
    reset_i8288();
  #endif
  result = cpu_reset();
  if(result) {
    change_state(Execute);
  }
  return result;
}

// Server command - Cpu type
// Return the detected CPU type
bool cmd_cpu_type() {
  debug_cmd("CPU_TYPE", "In cmd");
  snprintf(LAST_ERR, MAX_ERR_LEN, "NO ERROR");
  SERIAL.write((uint8_t)CPU.cpu_type);
  return true;
}

// Server command - Cycle
// Execute a single CPU cycle
bool cmd_cycle() {
  cycle();
  return true;
}

// Server command - Load
// Load the specified register state into the CPU.
// This command takes 28 bytes, which correspond to the word values of each of the 14
// CPU registers.
// Registers should be loaded in the following order, little-endian:
//
// AX, BX, CX, DX, SS, SP, FLAGS, IP, CS, DS, ES, BP, SI, DI
bool cmd_load() {

  //Serial1.println(">> Got load!");
  snprintf(LAST_ERR, MAX_ERR_LEN, "NO ERROR");

  // Sanity check
  if(SERVER.cmd_byte_n < sizeof (registers_t)) {
    set_error("Not enough command bytes");
    return false;
  }

  // Write raw command bytes over register struct.
  // All possible bit representations are valid.
  uint8_t *read_p = (uint8_t *)&CPU.load_regs;

  for(size_t i = 0; i < sizeof (registers_t); i++ ) {
    *read_p++ = COMMAND_BUFFER[i];
  }

  patch_load_pgm(LOAD_PROGRAM, &CPU.load_regs);
  patch_brkem_pgm(EMU_ENTER_PROGRAM, &CPU.load_regs);

  Serial1.println("## Loaded CS ##");
  Serial1.println(CPU.load_regs.cs);
  LOAD_REGISTERS.flags &= CPU_FLAG_DEFAULT_CLEAR;
  LOAD_REGISTERS.flags |= CPU_FLAG_DEFAULT_SET;

  #if EMULATE_8288
    reset_i8288();
  #endif
  bool result = cpu_reset();
  if(!result) {
    //set_error("Failed to reset CPU");
    return false;
  }

  change_state(CpuId);

  // Run CPU and wait for load to finish
  int load_timeout = 0;
  while(CPU.v_state != Execute) {
    cycle();
    load_timeout++;

    if(load_timeout > 300) {
      // Something went wrong in load program
       set_error("Load timeout");
       return false;
    }
  }

  #if LOAD_INDICATOR
    Serial1.print(".");
  #endif
  debug_proto("LOAD DONE");
  return true;
}

// Server command - ReadAddressLatch
// Read back the contents of the address latch as a sequence of 3 bytes (little-endian)
bool cmd_read_address_latch() {
  static char buf[7];

  #if MODE_ASCII  
    //buf[0] = 0;
    snprintf(
      buf, 7, 
      "%02X%02X%02X",
      (int)(CPU.address_latch & 0xFF),
      (int)((CPU.address_latch >> 8) & 0xFF),
      (int)((CPU.address_latch >> 16) & 0xFF)
    );
    SERIAL.print(buf);
  #else
    SERIAL.write((uint8_t)(CPU.address_latch & 0xFF));
    SERIAL.write((uint8_t)((CPU.address_latch >> 8) & 0xFF));
    SERIAL.write((uint8_t)((CPU.address_latch >> 16) & 0xFF));
  #endif

  return true;
}

// Server command - ReadAddress
// Read back the contents of the address bus as a sequence of 3 bytes (little-endian)
bool cmd_read_address() {
  read_address();
  static char buf[7];

  #if MODE_ASCII  
    //buf[0] = 0;
    snprintf(
      buf, 7, 
      "%02X%02X%02X",
      (int)(CPU.address_bus & 0xFF),
      (int)((CPU.address_bus >> 8) & 0xFF),
      (int)((CPU.address_bus >> 16) & 0xFF)
    );
    SERIAL.print(buf);
  #else
    SERIAL.write((uint8_t)(CPU.address_bus & 0xFF));
    SERIAL.write((uint8_t)((CPU.address_bus >> 8) & 0xFF));
    SERIAL.write((uint8_t)((CPU.address_bus >> 16) & 0xFF));
  #endif

  return true;
}

bool cmd_invalid() {
  Serial1.println("Called cmd_invalid!");
  return false;
}

// Server command - ReadStatus
// Return the value of the CPU status lines S0-S5 and QS0-QS1
bool cmd_read_status() {
  static char buf[3];
  read_status0();
  #if MODE_ASCII  
    snprintf(buf, 3, "%02X", CPU.status0);
    SERIAL.print(buf);
  #else
    SERIAL.write(CPU.status0);
  #endif
  return true;
}

// Server command - Read8288Command
bool cmd_read_8288_command() {
  static char buf[3];
  read_8288_command_bits();
  #if MODE_ASCII  
    snprintf(buf, 3, "%02X", CPU.command_bits);
    SERIAL.print(buf);
  #else
    SERIAL.write(CPU.command_bits);
  #endif
  return true;
}

// Server command - Read8288Control
bool cmd_read_8288_control() {
  static char buf[3];
  read_8288_control_bits();
  #if MODE_ASCII  
    snprintf(buf, 3, "%02X", CPU.control_bits);
    SERIAL.print(buf);
  #else
    SERIAL.write(CPU.control_bits);
  #endif
  return true;
}

// Server command - ReadDataBus
bool cmd_read_data_bus() {
  static char buf[3];
  #if MODE_ASCII  
    snprintf(buf, 3, "%02X", CPU.data_bus);
    SERIAL.print(buf);
  #else
    SERIAL.write((uint8_t)CPU.data_bus);
    SERIAL.write((uint8_t)(CPU.data_bus >> 8));
  #endif

  return true;
}

// Server command - WriteDataBus
// Takes an argument of 2 bytes.
// Sets the data bus to the provided value. On 8-bit CPUs the upper byte is ignored.
// This should not be called for CODE fetches after we have called cmd_prefetch_store(), 
// unless a flow control operation occurs that flushes the queue and returns us to 
// within original program boundaries.
bool cmd_write_data_bus() {
  if (CPU.bus_state_latched == CODE) {
    // We've just been instructed to write a normal fetch byte to the bus.
    // If we were prefetching the store program, reset this status as a queue
    // flush must have executed (or we goofed up...)
    CPU.prefetching_store = false;
    CPU.s_pc = 0;
  }

  CPU.data_bus = (uint16_t)COMMAND_BUFFER[0];
  CPU.data_bus |= ((uint16_t)COMMAND_BUFFER[1] << 8);
  CPU.data_type = DATA_PROGRAM;
  return true;
}

// Server command - PrefetchStore
// Instructs the CPU server to load the next byte of the Store (or EmuExit) program early
// Should be called in place of cmd_write_data_bus() by host on T3/TwLast when 
// program bytes have been exhausted.
// (When we are prefetching past execution boundaries during main program execution)
bool cmd_prefetch_store() {

  if (CPU.in_emulation) {
    // Prefetch the EmuExit program
    if (CPU.s_pc >= sizeof EMU_EXIT_PROGRAM) {
      set_error("EmuExit program underflow");
      return false;
    }

    #if DEBUG_STORE
      Serial1.print("## PREFETCH_EMU_EXIT: s_pc: ");
    #endif

    CPU.prefetching_store = true;
    CPU.data_bus = read_program(EMU_EXIT_PROGRAM, &CPU.s_pc, CPU.address_latch, CPU.data_width);
    CPU.data_type = DATA_PROGRAM_END;
  }
  else {
    // Prefetch the Store program
    if (CPU.s_pc >= sizeof STORE_PROGRAM) {
      set_error("Store program underflow");
      return false;
    }

    #if DEBUG_STORE
      Serial1.print("## PREFETCH_STORE: s_pc: ");
    #endif

    CPU.prefetching_store = true;
    CPU.data_bus = read_program(STORE_PROGRAM, &CPU.s_pc, CPU.address_latch, CPU.data_width);
    CPU.data_type = DATA_PROGRAM_END;
  }

  #if DEBUG_STORE
    Serial1.print(CPU.s_pc);
    Serial1.print(" addr: ");
    Serial1.print(CPU.address_latch, 16);
    Serial1.print(" data: ");
    Serial1.println(CPU.data_bus, 16);
  #endif

  return true;
}

// Server command - Finalize
// Sets the data bus flag to DATA_PROGRAM_END, so that the Execute state can terminate
// on the next instruction queue fetch
bool cmd_finalize() {
  if(CPU.v_state == Execute) {
    change_state(ExecuteFinalize);

    // Wait for execute done state
    int execute_ct = 0;
    int timeout = FINALIZE_TIMEOUT;
    if (CPU.in_emulation) {
      // We need more time to exit emulation mode
      timeout = FINALIZE_EMU_TIMEOUT;
    }
    while(CPU.v_state != ExecuteDone) {
      cycle();
      execute_ct++;

      

      if(execute_ct > timeout) {
        set_error("cmd_finalize(): state timeout");
        return false;
      }
    }
    return true;
  }
  else {
    error_beep();
    set_error("cmd_finalize(): wrong state: ");
    Serial1.println(CPU.v_state);
    return false;
  }
}

// Server command - BeginStore
// Execute state must be in ExecuteDone before intiating BeginStore command
//
bool cmd_begin_store(void) {
  /*
  char err_msg[30];

  // Command only valid in ExecuteDone state
  if(CPU.v_state != ExecuteDone) {
    snprintf(err_msg, 30, "BeginStore: Wrong state: %d ", CPU.v_state);
    set_error(err_msg);
    return false;
  }

  change_state(Store);
  */
  return true;
}

// Server command - Store
// 
// Returns values of registers in the following order, little-endian
// AX, BX, CX, DX, SS, SP, FLAGS, IP, CS, DS, ES, BP, SI, DI
// Execute state must be in StoreDone before executing Store command
bool cmd_store(void) {

  #if DEBUG_STORE
    Serial1.print("## In STORE: s_pc is: ");
    Serial1.println(CPU.s_pc);
  #endif

  char err_msg[30];
  // Command only valid in Store
  if(CPU.v_state != ExecuteDone) {
    snprintf(err_msg, 30, "STORE: Wrong state: %d ", CPU.v_state);

    set_error(err_msg);
    return false;
  }

  change_state(Store);

  int store_timeout = 0;

  // Cycle CPU until Store complete
  while(CPU.v_state != StoreDone) {
    cycle();
    store_timeout++;

    if (store_timeout > 500) {
      Serial1.println("## STORE: Timeout! ##");
      snprintf(err_msg, 30, "StoreDone timeout.");
      error_beep();
      return false;
    }
  }

  #if DEBUG_EMU
    Serial1.print("## STORE: Flags are: ");
    Serial1.println(CPU.post_regs.flags, 16);
  #endif

  // Dump final register state to Serial port
  uint8_t *reg_p = (uint8_t *)&CPU.post_regs;
  for(size_t i = 0; i < sizeof CPU.post_regs; i++ ) {
    SERIAL.write(reg_p[i]);
  }

  #if STORE_INDICATOR
    Serial1.print("?");
  #endif
  change_state(Done);
  return true;
}


// Server command - QueueLen
// Return the length of the instruction queue in bytes
bool cmd_queue_len(void) {

  SERIAL.write((uint8_t)CPU.queue.len);
  return true;
}

// Server command - QueueBytes
// Return the contents of the instruction queue, from 0-6 bytes.
bool cmd_queue_bytes(void) {

  for(size_t i = 0; i < CPU.queue.len; i++ ) {
    SERIAL.write(read_queue(i));
  }
  return true;
}

// Server command - Write pin
// Sets the value of the specified CPU input pin
bool cmd_write_pin(void) {

  uint8_t pin_idx = COMMAND_BUFFER[0];
  uint8_t pin_val = COMMAND_BUFFER[1] & 0x01;

  if(pin_idx < sizeof WRITE_PINS) {
    uint8_t pin_no = WRITE_PINS[pin_idx];

    switch(pin_no) {
      case READY_PIN:
        WRITE_READY_PIN(pin_val);
        break;

      case TEST_PIN:
        WRITE_TEST_PIN(pin_val);
        break;

      case INTR_PIN:
        WRITE_INTR_PIN(pin_val);
        break;

      case NMI_PIN:
        WRITE_NMI_PIN(pin_val);
        break;
      
      default:
        error_beep();
        return false;
    }
    return true;
  }
  else {
    // Invalid pin 
    error_beep();
    return false;
  }
}

// Server command - Read pin
bool cmd_read_pin(void) {
  // Not implemented
  SERIAL.write((uint8_t)0);
  return true;
}

// Server command - Get program state
bool cmd_get_program_state(void) {
  SERIAL.write((uint8_t)CPU.v_state);
  return true;
}

// Server command - Get last error
bool cmd_get_last_error(void) {
  SERIAL.write(LAST_ERR);
  return true;
}

// Server command - Get Cycle State
// A combination of all the status info typically needed for a single cycle
// Returns 4 bytes
bool cmd_get_cycle_state(void) {
  read_status0();
  read_8288_command_bits();
  read_8288_control_bits();
  uint8_t byte0 = ((uint8_t)CPU.v_state & 0x0F) << 4;
  byte0 |= (CPU.control_bits & 0x0F);

  SERIAL.write(byte0);
  SERIAL.write(CPU.status0);
  SERIAL.write(CPU.command_bits);
  SERIAL.write(uint8_t(CPU.data_bus & 0xFF));
  SERIAL.write(uint8_t(CPU.data_bus >> 8));
  return true;
}

// Server command - Cycle and Get Cycle State
// Cycle the CPU + return A combination of all the status info typically needed for a single cycle
// Returns 4 bytes
bool cmd_cycle_get_cycle_state(void) {
  cycle();
  cmd_get_cycle_state();
  return true;
}

// Server command - Enter emulation mode
bool cmd_emu8080(void) {
  if ((CPU.cpu_type == necV20) || (CPU.cpu_type == necV30)) {
    // Simply toggle the emulation flag
    CPU.do_emulation = true;
    #if DEBUG_EMU 
      Serial1.println("## cmd_emu8080(): Enabling 8080 emulation mode! ##");
    #endif
    return true;
  }
  // Unsupported CPU!
  #if DEBUG_EMU 
    Serial1.println("## cmd_emu8080(): Bad CPU type ## ");
  #endif
  return false;
}

void patch_vector_pgm(uint8_t *pgm, uint16_t seg) {
  *((uint16_t *)&pgm[3]) = seg;
}

void patch_load_pgm(uint8_t *pgm, registers_t *reg) {
  *((uint16_t *)pgm) = reg->flags;
  *((uint16_t *)&pgm[LOAD_BX]) = reg->bx;
  *((uint16_t *)&pgm[LOAD_CX]) = reg->cx;
  *((uint16_t *)&pgm[LOAD_DX]) = reg->dx;
  *((uint16_t *)&pgm[LOAD_SS]) = reg->ss;
  *((uint16_t *)&pgm[LOAD_DS]) = reg->ds;
  *((uint16_t *)&pgm[LOAD_ES]) = reg->es;
  *((uint16_t *)&pgm[LOAD_SP]) = reg->sp;
  *((uint16_t *)&pgm[LOAD_BP]) = reg->bp;
  *((uint16_t *)&pgm[LOAD_SI]) = reg->si;
  *((uint16_t *)&pgm[LOAD_DI]) = reg->di;
  *((uint16_t *)&pgm[LOAD_AX]) = reg->ax;
  *((uint16_t *)&pgm[LOAD_IP]) = reg->ip;
  *((uint16_t *)&pgm[LOAD_CS]) = reg->cs;
}

void patch_brkem_pgm(uint8_t *pgm, registers_t *regs) {
  static char buf[20];
  #if DEBUG_EMU 
    Serial1.println("## Patching BRKEM program ##");
    snprintf(buf, 20, 
      "CS: %04X IP: %04X",
      regs->cs,
      regs->ip);
    Serial1.println(buf);
  #endif
  uint16_t *word_ptr = (uint16_t *)pgm;
  *word_ptr++ = regs->ip;
  *word_ptr = regs->cs;
}

void print_registers(registers *regs) {
  static char buf[130];
  static char flag_buf[17];

  if(!regs) {
    return;
  }

  snprintf(buf, 130, 
    "AX: %04x BX: %04x CX: %04x DX: %04x\n"
    "SP: %04x BP: %04x SI: %04x DI: %04x\n"
    "CS: %04x DS: %04x ES: %04x SS: %04x\n"
    "IP: %04x\n"
    "FLAGS: %04x",
    regs->ax, regs->bx, regs->cx, regs->dx,
    regs->sp, regs->bp, regs->si, regs->di,
    regs->cs, regs->ds, regs->es, regs->ss,
    regs->ip,
    regs->flags );

  Serial1.println(buf);

  // Expand flag info
  uint16_t f = regs->flags;
  char c_chr = CPU_FLAG_CARRY & f ? 'C' : 'c';
  char p_chr = CPU_FLAG_PARITY & f ? 'P' : 'p';
  char a_chr = CPU_FLAG_AUX_CARRY & f ? 'A' : 'a';
  char z_chr = CPU_FLAG_ZERO & f ? 'Z' : 'z';
  char s_chr = CPU_FLAG_SIGN & f ? 'S' : 's';
  char t_chr = CPU_FLAG_TRAP & f ? 'T' : 't';
  char i_chr = CPU_FLAG_INT_ENABLE & f ? 'I' : 'i';
  char d_chr = CPU_FLAG_DIRECTION & f ? 'D' : 'd';
  char o_chr = CPU_FLAG_OVERFLOW & f ? 'O' : 'o';
  
  snprintf(
    flag_buf, 17,
    "1111%c%c%c%c%c%c0%c0%c1%c",
    o_chr, d_chr, i_chr, t_chr, s_chr, z_chr, a_chr, p_chr, c_chr
  );

  Serial1.print("FLAGSINFO: ");
  Serial1.println(flag_buf);
}

void print_cpu_state() {
  const size_t buf_len = 90;
  static char buf[buf_len];
  const size_t op_len = 9; //(4 + 4 + 1)
  static char op_buf[op_len];
  static char q_buf[15];
  static char data_buf[5];
  size_t bus_str_width = 0;

  const char *ale_str = READ_ALE_PIN ? "A:" : "  ";
  
  char rs_chr = !READ_MRDC_PIN ? 'R' : '.';
  char aws_chr = !READ_AMWC_PIN ? 'A' : '.';
  char ws_chr = !READ_MWTC_PIN ? 'W' : '.';
  
  char ior_chr = !READ_IORC_PIN ? 'R' : '.';
  char aiow_chr = !READ_AIOWC_PIN ? 'A' : '.';
  char iow_chr = !READ_IOWC_PIN ? 'W' : '.';

  char intr_chr = '.';
  char inta_chr = '.';
  char bhe_chr = !READ_BHE_PIN ? 'B' : '.';

  char v_chr = MACHINE_STATE_CHARS[(size_t)CPU.v_state];
  uint8_t q = (CPU.status0 >> 6) & 0x03;
  char q_char = QUEUE_STATUS_CHARS[q];
  char s = CPU.status0 & 0x07;

  // Set the bus string width
  if (CPU.data_width == BusWidthEight) {
    bus_str_width = 2;
  }
  else {
    bus_str_width = 4;
  }

  // Get segment from S3 & S4
  const char *seg_str = "  ";
  if(CPU.bus_cycle != T1) {
    // Status is not avaialble on T1 because address is latched
    uint8_t seg = ((CPU.status0 & 0x18) >> 3) & 0x03;
    seg_str = SEGMENT_STRINGS[(size_t)seg];
  }

  // Draw some sad ascii representation of bus transfers
  char *st_str = "  ";
  switch(CPU.bus_cycle) {
      case T1:
        if((CPU.bus_state != PASV) && (CPU.bus_state != HALT) && (CPU.bus_state != IRQA)) {
          // Begin a bus state
          st_str = "\\ ";
        }
        break;
      case T2: // FALLTHRU
      case T3: // FALLTHRU
      case TW:  
        // Continue a bus state
        st_str = " |";
        break;
      case T4:
        // End a bus state
        st_str = "/ ";
        break;
  }

  // Make data bus string and set r/w indicators based on bus size
  data_buf[5] = {' '};
  char *rd_str = "r";
  char *wr_str = "w";
  if (CPU.data_width == EightLow) {
    // Write two hex digits, 0-padded
    snprintf(data_buf, 5, "%4.2X", (uint8_t)CPU.data_bus);
  }
  else if (CPU.data_width == EightHigh) {
    // Write two hex digits, 0-padded, left-aligned in 4-character field
    snprintf(data_buf, 5, "%-4.2X", (uint8_t)(CPU.data_bus >> 8));
  }
  else {
    rd_str = "R";
    wr_str = "W";
    // Write four hex digits, 0-padded
    snprintf(data_buf, 5, "%04X", CPU.data_bus);
  }

  // Make string for bus reads and writes
  op_buf[0] = 0;
  if ((!READ_MRDC_PIN || !READ_IORC_PIN) && CPU.bus_state == PASV) {
      snprintf(op_buf, op_len, "%s-> %s", rd_str, data_buf);
  } else if (!READ_MWTC_PIN || !READ_IOWC_PIN) {
      snprintf(op_buf, op_len, "<-%s %s", wr_str, data_buf);
  } else {
      snprintf(op_buf, op_len, "%*s", (int)(4 + bus_str_width), "");
  }

  const char *q_str = queue_to_string();

  const char *t_str;
  if ((CPU.bus_cycle == T1) && (CPU.bus_state == PASV)) {
    // Convert T1 to Ti when passive bus
    t_str = "Ti";
  }
  else {
    t_str = CYCLE_STRINGS[(size_t)CPU.bus_cycle];

  }

  snprintf(
    buf, 
    buf_len, 
    "%08ld %c %s[%05lX][%05lX] %2s M:%c%c%c I:%c%c%c P:%c%c%c %-4s %s %2s %8s | %c%d [%-*s]", 
    CYCLE_NUM, 
    v_chr, 
    ale_str,
    //read_address(), 
    CPU.address_latch,
    peek_address(),
    seg_str,
    rs_chr, aws_chr, ws_chr,
    ior_chr, aiow_chr, iow_chr,
    intr_chr, inta_chr, bhe_chr,
    BUS_STATE_STRINGS[(size_t)CPU.bus_state],
    t_str, 
    st_str,
    op_buf,
    q_char,
    CPU.queue.len,
    CPU.queue.size * 2,
    q_str
  );

  Serial1.print(buf);

  if(q == QUEUE_FIRST) {
    // First byte of opcode read from queue. Decode it to opcode
    snprintf(q_buf, 15, " <-q %02X %s", CPU.qb, get_opcode_str(CPU.opcode, 0, false));
    Serial1.print(q_buf);
  }
  else if(q == QUEUE_SUBSEQUENT) {
    if(!CPU.in_emulation && IS_GRP_OP(CPU.opcode) && CPU.q_fn == 1) {
      // Modrm was just fetched for a group opcode, so display the mnemonic now
      snprintf(q_buf, 15, " <-q %02X %s", CPU.qb, get_opcode_str(CPU.opcode, CPU.qb, true));
    }
    else {
      snprintf(q_buf, 15, " <-q %02X", CPU.qb);
    }
    Serial1.print(q_buf);
  }

  Serial1.println("");
}

void change_state(machine_state_t new_state) {
  switch(new_state) {
    case Reset:
      CPU.doing_reset = true;    
      CPU.cpuid_counter = 0;
      CPU.cpuid_queue_reads = 0;
      CPU.v_pc = 0;
      CPU.s_pc = 0;
      break;
    case CpuId: 
      CPU.doing_reset = false;
      CPU.doing_id = true;
      CPU.cpuid_counter = 0;
      CPU.cpuid_queue_reads = 0;
      CPU.v_pc = 0;
      break;
    case JumpVector:
      CPU.doing_reset = false;
      CPU.v_pc = 0;
      break;
    case Load:
      // Set v_pc to 2 to skip flag bytes
      CPU.v_pc = 2;
      break;
    case LoadDone:
      break;
    case EmuEnter:
      CPU.stack_r_op_ct = 0;
      CPU.stack_w_op_ct = 0;
      // Set v_pc to 4 to skip IVT segment:offset
      CPU.v_pc = 4;
      break;
    case Execute:
      CPU.v_pc = 0;
      CPU.s_pc = 0;
      if (CPU.do_emulation) {
        // Set v_pc to 4 to skip IVT segment:offset
        CPU.s_pc = 4;
      }
      
      break;
    case ExecuteFinalize:
      break;
    case ExecuteDone:
      break;
    case EmuExit:
      CPU.stack_r_op_ct = 0;
      CPU.stack_w_op_ct = 0;
      CPU.v_pc = 0;
      break;
    case Store:
      // Take a raw uint8_t pointer to the register struct. Both x86 and Arduino are little-endian,
      // so we can write raw incoming data over the struct. Faster than logic required to set
      // specific members. 
      CPU.readback_p = (uint8_t *)&CPU.post_regs;      
      break;
    case StoreDone:
      break;
    case Done:
      break;
  }

  uint32_t state_end_time = micros();

  #if DEBUG_STATE
    // Report time we spent in the previous state.
    if(CPU.state_begin_time != 0) {
      uint32_t elapsed = state_end_time - CPU.state_begin_time;
      Serial1.print("## Changing to state: ");
      Serial1.print(MACHINE_STATE_STRINGS[(size_t)new_state]);
      Serial1.print(". Spent (");
      Serial1.print(elapsed);
      Serial1.println(") us in previous state. ##");
    }
  #endif

  CPU.state_begin_time = micros();
  CPU.v_state = new_state;
}

// Emulate a code fetch from the specified program.
uint16_t read_program(const uint8_t *program, uint16_t *pc, uint32_t address, data_width_t width) {

  uint16_t data = 0;

  if (width == EightLow) {
    data = program[(*pc)++];
  }
  else if (width == EightHigh) {
    // TODO: bounds checks
    data = program[(*pc) + 1];
  }
  else {
    // 16-bit read. 

    if ((address & 1) == 0) {
      // Even address
      //Serial1.println("## Even read ##");
      data = program[(*pc)++];
      data |= ((uint16_t)program[(*pc)++]) << 8;
    }
    else {
      // Odd address. 
      //Serial1.println("## Odd read ##");
      if ((*pc) > 0) {
        // This byte doesn't really matter, but we can simulate fetching more realistically by including it.
        // If this happens to be the start of the program though, it will just have to be 0.
        data = program[(*pc) - 1];
      }
      data |= ((uint16_t)program[(*pc)++]) << 8;
    }
  }

  return data;
}

// Simulate fetching NOPs based on bus width.
uint16_t read_nops(data_width_t width) {
  if (width == EightLow) {
    return 0x90;
  }
  else {
    return 0x9090;
  }
}

void set_data_bus_width() {
  if (!READ_BHE_PIN) {
    if ((CPU.address_latch & 1) == 0) {
      // BHE is active, and address is even. Bus width is 16.
      CPU.data_width = Sixteen;
    }
    else {
      // BHE is active, and address is odd. Bus width is EightHigh.
      CPU.data_width = EightHigh;
    }
  }
  else {
    // If BHE is inactive, then we can't read an even address. So this must be 
    // EightLow.
    CPU.data_width = EightLow;
  }
}

void cycle() {

  // First, tick the CPU and increment cycle count
  clock_tick();
  CYCLE_NUM++;
  if(CYCLE_NUM == 0) {
    // overflow
    CYCLE_NUM_H++;
  }

  CPU.cpuid_counter++;

  // Read the CPU status pins
  read_status0();

  // bus_state is the instantaneous state per cycle. May not always be valid. 
  // for state of the current bus transfer use bus_state_latched
  CPU.bus_state = (s_state)(CPU.status0 & 0x07);

  // Extract QS0-QS1 queue status
  uint8_t q = (CPU.status0 >> 6) & 0x03;
  CPU.qb = 0xFF;
  CPU.q_ff = false;

  if(READ_ALE_PIN) {
    // ALE signals start of bus cycle, so set cycle to t1.
    CPU.bus_cycle = T1;
    // Address lines are only valid when ALE is high, so latch address now.
    latch_address();
    CPU.bus_state_latched = CPU.bus_state; 
    //Serial1.print("## LATCHED ADDRESS: ");
    //Serial1.println(CPU.address_latch);
  }

  // Operate current T-state
  switch(CPU.bus_cycle) {
    case T1:
      // Set the data bus width
      set_data_bus_width();
      break;

    case T2:
      break;

    case T3:
      break;

    case T4:
      // Did we complete a code fetch? If so, increment queue len
      if(CPU.bus_state_latched == CODE) {
        //Serial1.print("## T4 of CODE fetch. Q is: ");
        //Serial1.println(q);

        if(q == QUEUE_FLUSHED) {
          Serial1.println("## Queue flush during T4. Supressing queue push.");
          if(CPU.queue.len < CPU.queue.size) {
            push_queue(CPU.data_bus, CPU.data_type, CPU.data_width);
          }
          else {
            // Shouldn't be here
            Serial1.println("## Error: Invalid Queue Length++ ##");
          }
        }
        else {
          if(CPU.queue.len < CPU.queue.size) {
            push_queue(CPU.data_bus, CPU.data_type, CPU.data_width);
          }
          else {
            // Shouldn't be here
            Serial1.println("## Error: Invalid Queue Length++ ##");
          }
        }
      }
      CPU.bus_state_latched = PASV;
      break;
  }

  // Handle queue activity
  if((q == QUEUE_FIRST) || (q == QUEUE_SUBSEQUENT)) {
    // We fetched a byte from queue last cycle
    if(CPU.queue.len > 0 ) {
      pop_queue(&CPU.qb, &CPU.qt);
      if(q == QUEUE_FIRST) {
        // Set flag for first instruction byte fetched
        CPU.q_ff = true;
        CPU.q_fn = 0; // First byte of instruction
        CPU.opcode = CPU.qb;
        CPU.mnemonic = get_opcode_str(CPU.opcode, 0, false);
        #if DEBUG_INSTR
          if (!IS_GRP_OP(CPU.opcode)) {
            Serial1.print("INST: ");
            Serial1.println(CPU.mnemonic);
          }
          else {
            Serial1.println("INST: Decoding GRP...");
          }
        #endif
      }
      else {
        if(IS_GRP_OP(CPU.opcode) && CPU.q_fn == 1) {
          CPU.mnemonic = get_opcode_str(CPU.opcode, CPU.qb, true);
          #if DEBUG_INSTR 
            Serial1.print("INST: ");
            Serial1.println(CPU.mnemonic);
          #endif
        }
        // Subsequent byte of instruction fetched
        CPU.q_fn++;
      }
    }
    else {
      Serial1.println("## Error: Invalid Queue Length-- ##");
    }
  }
  else if(q == QUEUE_FLUSHED) {
    // Queue was flushed last cycle.

    // Warn if queue is flushed during CODE cycle.
    if (CPU.bus_state_latched == CODE) {
        Serial1.print("## FLUSH during CODE fetch! t-state: ");
        switch (CPU.bus_cycle) {
          case T1:
            Serial1.println("T1");
            break;
          case T2:
            Serial1.println("T2");
            break;
          case T3:
            Serial1.println("T3");
            break;
          case T4:
            Serial1.println("T4");
            break;
        }
    }

    // The queue is flushed once during store program, so we need to adjust s_pc 
    // by the length of the queue when it was flushed or else we'll skip bytes
    // of the store program.
    if (CPU.s_pc > 0) {

      if (CPU.s_pc < 4) {
        #if DEBUG_STORE
          Serial1.println("## FLUSHed STORE bytes (early): Reset s_pc");
        #endif
        CPU.s_pc = 0;
      }
      else if (CPU.s_pc >= CPU.queue.len) {
        uint16_t pc_adjust = (uint16_t)CPU.queue.len;

        if ((pc_adjust & 1) && (CPU.width == BusWidthSixteen)) {
          // If we have an odd queue length and 16-bit fetches, account for one more byte
          pc_adjust++;
        }
        CPU.s_pc -= pc_adjust;
        #if DEBUG_STORE
          Serial1.print("## FLUSHed STORE bytes: Adjusted s_pc by: ");
          Serial1.print(pc_adjust);
          Serial1.print(" new s_pc: ");
          Serial1.println(CPU.s_pc);
        #endif
      }
      else {
        #if DEBUG_STORE
          Serial1.print("## FLUSHed STORE bytes: Reset s_pc on flush");
        #endif
        CPU.s_pc = 0;
      }
    }

    empty_queue();

    #if TRACE_QUEUE
      SERIAL.println("## Queue Flushed ##");
      SERIAL.print("## PC: ");
      SERIAL.println(CPU.v_pc);
    #endif
  }

  switch(CPU.v_state) {


    case CpuId:
      // We are executing the CPU ID routine.
      if(!READ_MRDC_PIN) {
        // CPU is reading (MRDC active-low)      
        if(CPU.bus_state == CODE) {    
          // We are reading a code byte
          if(CPU.v_pc < sizeof JUMP_VECTOR) {
            // Feed CPU ID instruction to CPU.
            CPU.data_bus = read_program(CPUID_PROGRAM, &CPU.v_pc, CPU.address_latch, CPU.data_width);
            CPU.data_type = DATA_PROGRAM;
            data_bus_write(CPU.data_bus, CPU.data_width);
            // Immediately change to next state.
            #if USE_LOAD_SEG  
              change_state(JumpVector);
            #else
              change_state(Load);
            #endif
          }
        }
      }
      break;

    case JumpVector:
      // We are executing the initial jump from the reset vector FFFF:0000.
      // This is to avoid wrapping effective address during load procedure.
      // Optional - disable in header

      // If previous state was CpuId, then reset the utility timer on the first queue read.
      if (CPU.doing_id && (q == QUEUE_FIRST)) {
        if (CPU.cpuid_queue_reads == 0) {
          #if TRACE_ID
            Serial1.println("## Starting CPUID counter! ##");
          #endif
          CPU.cpuid_counter = 0;
        }
        else {
            #if TRACE_ID
              Serial1.print("## CPUID counter stopped at: ");
              Serial1.print(CPU.cpuid_counter);          
              Serial1.println(" ##");
            #endif
            CPU.doing_id = false;
            detect_cpu_type(CPU.cpuid_counter);
        }
        CPU.cpuid_queue_reads++;
      }

      if(!READ_MRDC_PIN) {
        // CPU is reading (MRDC active-low)      
        if(CPU.bus_state == CODE) {    
          // We are reading a code byte.
          if(CPU.v_pc < sizeof JUMP_VECTOR) {
            // Feed jump instruction to CPU

            //CPU.data_bus = JUMP_VECTOR[CPU.v_pc];
            CPU.data_bus = read_program(JUMP_VECTOR, &CPU.v_pc, CPU.address_latch, CPU.data_width);
            CPU.data_type = DATA_PROGRAM;
          }
          else {
            // Ran out of program, so return NOP. Doesn't matter what we feed
            // as queue will be reset.
            CPU.data_bus = read_nops(CPU.data_width);
            CPU.data_type = DATA_PROGRAM_END;
          }
          data_bus_write(CPU.data_bus, CPU.data_width);
        }
      }        

      if(READ_ALE_PIN) {
        // Jump is finished on first address latch of LOAD_SEG:0
        uint32_t dest = calc_flat_address(LOAD_SEG, 0);
        if(dest == CPU.address_latch) {
          // Transition to Load state.
          change_state(Load);
          break;
        }
      }      
      break;

    case Load:
      // We are executing the register load routine.

      if(!READ_MRDC_PIN) {
        // CPU is reading (MRDC active-low)
        if(CPU.bus_state == CODE) {      
          // We are reading a code byte
          if(CPU.v_pc < sizeof LOAD_PROGRAM) {
            // Feed load program to CPU
            CPU.data_bus = read_program(LOAD_PROGRAM, &CPU.v_pc, CPU.address_latch, CPU.data_width);
            CPU.data_type = DATA_PROGRAM;
          }
          else {
            // Ran out of program, so return NOP. JMP cs:ip will actually fetch once before SUSP,
            // so we wil see this NOP prefetched.
            #if (DATA_BUS_SIZE == 1)
              CPU.data_bus = OPCODE_NOP;
            #else
              CPU.data_bus = OPCODE_DOUBLENOP;
            #endif

            CPU.data_type = DATA_PROGRAM_END;
            //change_state(LoadDone);
          }
          data_bus_write(CPU.data_bus, CPU.data_width);
        }
        
        if(CPU.bus_state == MEMR) {
          // We are reading a memory byte
          // This should only occur during Load when flags are popped from 0:0
          if(CPU.address_latch < 0x00002 ) {
            // First two bytes of LOAD_PROGRAM were patched with flags
            uint16_t dummy_pc = (uint16_t)CPU.address_latch;
            CPU.data_bus = read_program(LOAD_PROGRAM, &dummy_pc, CPU.address_latch, CPU.data_width);
            CPU.data_type = DATA_PROGRAM;
            data_bus_write(CPU.data_bus, CPU.data_width);
          }
          else {
            // Unexpected read above address 0x00001
            Serial1.println("## INVALID MEM READ DURING LOAD ##");
          }
        }
      } 

      if (q == QUEUE_FLUSHED) {
        // Queue flush after final jump triggers next state.
        change_state(LoadDone);
      }
      break;

    case LoadDone:
      // LoadDone is triggered by the queue flush following the jump in Load.
      // We wait for the next ALE and begin Execute.

      #if DEBUG_LOAD_DONE
        Serial1.print("LoadDone: READ_ALE_PIN=");
        Serial1.print(READ_ALE_PIN);
        Serial1.print(" CPU.bus_state=");
        Serial1.println(BUS_STATE_STRINGS[(size_t)CPU.bus_state]);
      #endif

      if(READ_ALE_PIN && (CPU.bus_state == CODE)) {
        // First bus cycle of the instruction to execute. Transition to Execute or EmuEnter as appropriate.
        if (CPU.do_emulation && !CPU.in_emulation) {
          change_state(EmuEnter);
        }
        else {
          change_state(Execute);
        }
      }
      break;

    case EmuEnter:
      // We are executing the BRKEM routine.

      if(!READ_MRDC_PIN) {
        // CPU is reading (MRDC active-low)
        if(CPU.bus_state == CODE) {      
          // We are reading a code byte
          if(CPU.v_pc < sizeof EMU_ENTER_PROGRAM) {
            // Feed load program to CPU
            CPU.data_bus = read_program(EMU_ENTER_PROGRAM, &CPU.v_pc, CPU.address_latch, CPU.data_width);
            CPU.data_type = DATA_PROGRAM;
          }
          else {
            // Ran out of program, so return NOP. 
            CPU.data_bus = OPCODE_DOUBLENOP;
            CPU.data_type = DATA_PROGRAM_END;
            //change_state(LoadDone);
          }
          data_bus_write(CPU.data_bus, CPU.data_width);
        }
        
        if(CPU.bus_state == MEMR) {
          // We are reading from memory
          // This will occur when BRKEM reads the emulation segment vector
          uint32_t vector_base = BRKEM_VECTOR * 4;
          if((CPU.address_latch >= vector_base ) && (CPU.address_latch < vector_base + 4)) {
            if (CPU.address_latch < (vector_base + 2)) {
              // Reading offset, feed IP
              #if DEBUG_EMU
                Serial1.println("## Reading BRKEM offset! ##");
              #endif
            }
            else {
              // Reading segment
              #if DEBUG_EMU
                Serial1.println("## Reading BRKEM segment! ##");
              #endif
            }
            // Feed a dummy pc variable to read_program - the actual address is determined from 
            // the address latch
            uint16_t dummy_pc = (uint16_t)(CPU.address_latch - vector_base);
            CPU.data_bus = read_program(EMU_ENTER_PROGRAM, &dummy_pc, CPU.address_latch, CPU.data_width);
            CPU.data_type = DATA_PROGRAM;
            data_bus_write(CPU.data_bus, CPU.data_width);
          }
          else {
            // Unexpected read above address 0x00001
            Serial1.println("## INVALID MEM READ DURING EMUENTER ##");
          }
        }
      } 

      if(!READ_MWTC_PIN) {
        if (CPU.width == BusWidthEight) {
          // Flags will be read in two operations
          if (CPU.stack_w_op_ct == 0) {
            #if DEBUG_EMU
              Serial1.println("## Reading BRKEM flag push (1/2)! ##");
            #endif
            CPU.pre_emu_flags = (uint16_t)data_bus_read_byte();
          }
          else if (CPU.stack_w_op_ct == 1) {
            #if DEBUG_EMU
              Serial1.println("## Reading BRKEM flag push (2/2)! ##");
            #endif      
            CPU.pre_emu_flags |= ((uint16_t)data_bus_read_byte() << 8);
          }
          CPU.stack_w_op_ct++;
        }
        else {
          // Flags will be read in one operation
          if (CPU.stack_w_op_ct == 0) {
            #if DEBUG_EMU
              Serial1.println("## Reading BRKEM flag push! ##");
            #endif
            // CPU is writing to the data bus, latch value
            CPU.data_bus = data_bus_read(CPU.data_width);
            CPU.pre_emu_flags = CPU.data_bus;
          }
          CPU.stack_w_op_ct++;
        }
      }

      if (q == QUEUE_FLUSHED) {
        // Queue flush after final jump triggers next state.
        CPU.in_emulation = true;
        change_state(LoadDone);
      }

      break;

    // Unlike in run_program, the Execute state in cpu_server is entirely interactive based on 
    // commands from the client. 
    // This is to support interception of memory reads & writes as instructions execute and to allow
    // the client to query CPU state as it wishes per cpu cycle.
    // When done in the Execute state, a cpu client should execute the ExecuteFinalize command.
    // This is typically done when a CODE fetch occurs past the end of the provided program, although
    // other end conditions are possible.
    case Execute:
    
      if ((!READ_MRDC_PIN || !READ_IORC_PIN) && CPU.bus_state == PASV) {
        // CPU is reading from data bus. We assume that the client has called CmdWriteDataBus to set 
        // the value of CPU.data_bus. Write it.
        data_bus_write(CPU.data_bus, CPU.data_width);

        if ((CPU.bus_state_latched == CODE) && (CPU.prefetching_store)) {
          //CPU.s_pc++;
          #if DEBUG_STORE
            Serial1.print("STORE: Wrote STORE PGM BYTE to bus: ");
            Serial1.print(CPU.data_bus, 16);
            Serial1.print(" new s_pc: ");
            Serial1.println(CPU.s_pc);
          #endif
        }
      }
    
      if(!READ_MWTC_PIN || !READ_IOWC_PIN) {
        // CPU is writing to the data bus, latch value
        CPU.data_bus = data_bus_read(CPU.data_width);
      }
      
      break;

    // The ExecuteFinalize state is unique to the cpu_server. Since Execute is now an interactive state,
    // we need to be able to transition safely from Execute to Store. ExecuteState feeds the CPU
    // STORE program bytes flagged with DATA_PROGRAM_END and transitions to Store when one of those bytes
    // is fetched as the first byte of an instruction.
    case ExecuteFinalize:
      
      if (!READ_MRDC_PIN && CPU.bus_state == PASV) {
        // CPU is reading (MRDC active-low)
        if ((CPU.bus_state_latched == CODE) && (CPU.prefetching_store)) {
          // Since client does not cycle the CPU in this state, we have to fetch from the 
          // STORE or EMU_EXIT program ourselves

          const uint8_t *program = STORE_PROGRAM;
          uint16_t *pc_ptr = &CPU.s_pc;
          if (CPU.in_emulation) {
            program = EMU_EXIT_PROGRAM;
            pc_ptr = &CPU.v_pc;
          }
          
          CPU.data_bus = read_program(program, pc_ptr, CPU.address_latch, CPU.data_width);
          CPU.data_type = DATA_PROGRAM_END;
          data_bus_write(CPU.data_bus, CPU.data_width);
          #if DEBUG_STORE
            Serial1.print("ExecuteFinalize: Wrote next PGM word to bus: ");
            Serial1.print(CPU.data_bus, 16);
            Serial1.print(" new s_pc: ");
            Serial1.println(CPU.s_pc);
          #endif
        }
        else {
          data_bus_write(CPU.data_bus, CPU.data_width);
        }
      }

      if(CPU.q_ff && (CPU.qt == DATA_PROGRAM_END)) {
        // We read a flagged NOP, meaning the previous instruction has completed and it is safe to 
        // execute the Store routine.
        if (CPU.in_emulation) {
          change_state(EmuExit);
        }
        else {
          change_state(ExecuteDone);
        }
      }
      break;

    case EmuExit:
      if (!READ_MRDC_PIN) {
        // CPU is reading (MRDC active-low)
        if ((CPU.bus_state_latched == CODE) && (CPU.bus_state == PASV)) {
          // CPU is doing code fetch
          if(CPU.s_pc < sizeof EMU_EXIT_PROGRAM) {
            // Read code byte from EmuExit program
            CPU.data_bus = read_program(EMU_EXIT_PROGRAM, &CPU.s_pc, CPU.address_latch, CPU.data_width);
            #if DEBUG_EMU
              Serial1.print("## EMUEXIT: fetching byte: ");
              Serial1.print(CPU.data_bus, 16);
              Serial1.print(" new s_pc: ");
              Serial1.println(CPU.s_pc);
            #endif
            CPU.data_type = DATA_PROGRAM;
          }
          else {
            CPU.data_bus = OPCODE_DOUBLENOP;
            CPU.data_type = DATA_PROGRAM_END;
          }
          data_bus_write(CPU.data_bus, CPU.data_width);
        }

        if((CPU.bus_state_latched == MEMR) && (CPU.bus_state == PASV)) {
          // CPU is doing memory read
          // This will occur when RETEM pops IP, CS and Flags from the stack.

          if (CPU.width == BusWidthEight) {
            // Stack values will be read in two operations
            if (CPU.stack_r_op_ct == 0) {

            }
            else if (CPU.stack_r_op_ct == 1) {

            }
            else if (CPU.stack_r_op_ct == 2) {
              #if DEBUG_EMU
                Serial1.println("## Reading RETEM CS pop (1/2)! ##");
                Serial1.println(CPU.load_regs.cs);
              #endif              
              // Write the low byte of CS to the data bus
              data_bus_set_byte((uint8_t)(CPU.load_regs.cs));
            }
            else if (CPU.stack_r_op_ct == 3) {
              #if DEBUG_EMU
                Serial1.println("## Reading RETEM CS pop (2/2)! ##");
                Serial1.println(CPU.load_regs.cs);
              #endif  
              // Write the high byte of CS to the data bus
              data_bus_set_byte((uint8_t)(CPU.load_regs.cs >> 8));              
            }            
            else if (CPU.stack_r_op_ct == 4) {
              #if DEBUG_EMU
                Serial1.println("## Reading RETEM flag pop (1/2)! ##");
              #endif
              // Write the low byte of flags to the data bus
              data_bus_set_byte((uint8_t)(CPU.pre_emu_flags));
            }
            else if (CPU.stack_r_op_ct == 5) {
              #if DEBUG_EMU
                Serial1.println("## Reading RETEM flag pop (2/2)! ##");
              #endif      
              // Write the high byte of flags to the data bus
              data_bus_set_byte((uint8_t)(CPU.pre_emu_flags >> 8));
              // Exit emulation mode
              CPU.in_emulation = false;
              change_state(ExecuteFinalize);
            }
            else {
              // Not flags, just write 0's so we jump back to CS:IP 0000:0000
              CPU.data_bus = 0;
            }
            CPU.stack_r_op_ct++;
          }
          else {
            // Sixteen-bit data bus

            if (CPU.stack_r_op_ct == 0) {
              // IP is read in one operation
              #if DEBUG_EMU
                Serial1.println("## Reading RETEM IP pop! ##");
              #endif              
              CPU.data_bus = 0;
            }
            else if (CPU.stack_r_op_ct == 1) {
              // CS is read in one operation
              #if DEBUG_EMU
                Serial1.println("## Reading RETEM CS pop! ##");
                Serial1.println(CPU.load_regs.cs);
              #endif              
              // We can restore CS from the loaded registers since CS cannot be modified in 8080 emulation mode
              CPU.data_bus = CPU.load_regs.cs;
            }
            else if (CPU.stack_r_op_ct == 2) {
              // Flags will be read in one operation
              #if DEBUG_EMU
                Serial1.println("## Reading RETEM Flag pop! ##");
              #endif
              // CPU is writing to the data bus, latch value
              CPU.data_bus = CPU.pre_emu_flags;
              // Exit emulation mode
              CPU.in_emulation = false;
              change_state(ExecuteFinalize);
            }
            CPU.stack_r_op_ct++;
          }
          data_bus_write(CPU.data_bus, CPU.data_width);
        }
      }

      if (!READ_MWTC_PIN && (CPU.bus_state_latched == MEMW) && (CPU.bus_state == PASV)) {
        // CPU is writing. This should only happen during EmuExit when we PUSH PSW 
        // to save the 8080 flags. 

        CPU.data_bus = data_bus_read(CPU.data_width);

        if (CPU.data_width == BusWidthEight) {
          // 8-bit data bus
          if (CPU.stack_w_op_ct == 0) {
            // Flags will be in first byte written (second byte will be AL)
            #if DEBUG_EMU
              Serial1.println("## Capturing PUSH PSW stack write! ##");
            #endif
            CPU.emu_flags = (uint8_t)CPU.data_bus;
          }
          CPU.stack_w_op_ct++;
        }
        else {
          // 16-bit data bus
          if (CPU.stack_w_op_ct == 0) {
            // Flags were pushed in one operation. 
            #if DEBUG_EMU
              Serial1.println("## Capturing PUSH PSW stack write! ##");
            #endif            
            CPU.emu_flags = (uint8_t)CPU.data_bus;
          }
          CPU.stack_w_op_ct++;
        }
      }

      break;

    case ExecuteDone:
      // We sit in ExecuteDone state until the client requests a Store operation.
      // The client should not cycle the CPU in this state.
      if (!READ_MRDC_PIN && CPU.bus_state == PASV) {
        // CPU is reading (MRDC active-low)
        data_bus_write(CPU.data_bus, CPU.data_width);

        if ((CPU.bus_state_latched == CODE) && (CPU.prefetching_store)) {
          // Since client does not cycle the CPU in this state, we have to fetch from 
          // STORE program ourselves          

          CPU.data_bus = read_program(STORE_PROGRAM, &CPU.s_pc, CPU.address_latch, CPU.data_width);
          //CPU.data_bus = STORE_PROGRAM[CPU.s_pc++];
          CPU.data_type = DATA_PROGRAM_END;
          data_bus_write(CPU.data_bus, CPU.data_width);
          #if DEBUG_STORE
            Serial1.print("STORE: Wrote STORE PGM BYTE to bus (in EXECUTE_DONE): ");
            Serial1.print(CPU.data_bus, 16);
            Serial1.print(" new s_pc: ");
            Serial1.println(CPU.s_pc);
          #endif
        }
        else {
          Serial1.println("## Invalid condition: ExecuteDone without loading STORE");
          data_bus_write(CPU.data_bus, CPU.data_width);
        }
      }      
      break;

    case Store:
      // We are executing the Store program.
      if (!READ_MRDC_PIN && CPU.bus_state == PASV) {
        // CPU is reading
        
        if (CPU.bus_state_latched == CODE) {
          // CPU is doing code fetch
          if(CPU.s_pc < sizeof STORE_PROGRAM) {
            // Read code byte from store program
            //CPU.data_bus = STORE_PROGRAM[CPU.s_pc++];
            CPU.data_bus = read_program(STORE_PROGRAM, &CPU.s_pc, CPU.address_latch, CPU.data_width);
            #if DEBUG_STORE
              Serial1.print("STORE: fetching byte: ");
              Serial1.print(CPU.data_bus, 16);
              Serial1.print(" new s_pc: ");
              Serial1.println(CPU.s_pc);
            #endif
            CPU.data_type = DATA_PROGRAM;

          }
          else {
            CPU.data_bus = OPCODE_DOUBLENOP;
            CPU.data_type = DATA_PROGRAM_END;
          }
        }
        data_bus_write(CPU.data_bus, CPU.data_width);
      }

      // CPU is writing to memory address - this should only happen during readback when
      // the flags register is pushed to the stack (The only way to read the full flags)      
      if(!READ_MWTC_PIN) {
        CPU.data_bus = data_bus_read(CPU.data_width);

        // Store program sets up SS:SP as 0:4, so write should be to the first four memory
        // addresses, for pushing IP and FLAGS.
        if(CPU.address_latch < 0x00004) {

          #if DEBUG_STORE
            Serial1.println("## STORE Stack Push");
          #endif

          // Write flags and IP to the register struct
          if (CPU.data_width == EightLow) {
            #if DEBUG_EMU
              Serial1.print("## 8-bit flag read ##");
            #endif
            *CPU.readback_p = (uint8_t)CPU.data_bus;
            CPU.readback_p++;
          }
          else if (CPU.data_width == EightHigh) {
            // We shouldn't have unaligned stack access during STORE. Something has gone wrong.
            Serial1.println("## Bad Data Bus Width during Store: EightHigh");
          }
          else {
            // 16-bit data bus
            if((CPU.address_latch == 0x00002) && (CPU.do_emulation)) {
              // We ran a program in 8080 emulation. We want to substitute the flags 
              // captured in 8080 mode for the native flags now.
              CPU.data_bus = (CPU.data_bus & 0xFF00) | (uint16_t)CPU.emu_flags;
              #if DEBUG_EMU
                Serial1.print("## Substituting 8080 flags in stack read: ");
                Serial1.println(CPU.data_bus, 16);
              #endif
              
            }

            ptrdiff_t diff = (uint8_t *)&CPU.post_regs.flags - CPU.readback_p;            
            *((uint16_t *)CPU.readback_p) = CPU.data_bus;
            CPU.readback_p += 2;

            #if DEBUG_EMU
              uint16_t *flags_ptr = (uint16_t *)&CPU.post_regs.flags;          
              Serial1.print("## New flags are: ");
              Serial1.println(*flags_ptr, 16);
              Serial1.print("## Readback ptr diff: ");
              Serial1.println(diff);
            #endif
          }
        }
        else {
          // We shouldn't be writing to any other addresses, something wrong happened
          if (CPU.address_latch == 0x00004) {
            Serial1.println("## TRAP detected in Store operation! Invalid flags?");
          }

          Serial1.println("## INVALID STORE WRITE: ");
          Serial1.println(CPU.address_latch, HEX);
          set_error("Invalid store write");
          // TODO: handle error gracefully
        }
        #if DEBUG_STORE
          Serial1.print("## Store memory write: ");
          Serial1.println(CPU.data_bus, HEX);
        #endif
      }

      // CPU is writing to IO address - this indicates we are saving a register value. 
      // We structured the register struct in the right order, so we can overwrite it
      // directly.
      if(!READ_IOWC_PIN) {

        if(CPU.address_latch == 0xFD) {
          // Write to 0xFD indicates end of store procedure.
          
          // Adjust IP by offset of CALL instruction.
          #if DEBUG_STORE
            Serial1.print("## Unadjusted IP: ");
            Serial1.println(CPU.post_regs.ip, HEX);
          #endif            
          //CPU.post_regs.ip -= 0x24;
          CPU.post_regs.ip -= (0x24 + 6); // added 6 NOPs to start of STORE program
          
          change_state(StoreDone);
        }
        else {
          CPU.data_bus = data_bus_read(CPU.data_width);

          if (CPU.data_width == EightLow) {
            *CPU.readback_p = (uint8_t)CPU.data_bus;
            CPU.readback_p++;
          }
          else if (CPU.data_width == EightHigh) {
            Serial1.println("## Bad Data Bus Width during Store: EightHigh");
          }
          else {
            *(uint16_t *)CPU.readback_p = CPU.data_bus;
            CPU.readback_p += 2;
          }

          #if DEBUG_STORE
            Serial1.print("## Store IO write: ");
            Serial1.println(CPU.data_bus, HEX);
          #endif
        }
      }

    /*
    case Done:
      if(!READ_MRDC_PIN) {
        // CPU is reading
        
        if(CPU.bus_state == CODE) {
          // CPU is doing code fetch
          CPU.data_bus = 0x90;
          CPU.data_type = DATA_PROGRAM_END;
          data_bus_write(CPU.data_bus);
        }
        else {
          Serial1.print("**Unexpected read in DONE**");
        }        
      }    
      break;
      */

    break;
  }

  #if DEBUG_LOCK
    if (!READ_LOCK_PIN) {
      Serial1.println(">>> LOCK!");
    }
  #endif

  // Print instruction state if tracing is enabled
  switch(CPU.v_state) {
    case Reset:
      #if TRACE_RESET
        print_cpu_state();
      #endif
      break;
    case CpuId:
      #if TRACE_ID
        print_cpu_state();
      #endif
      break;      
    case JumpVector:
      #if TRACE_VECTOR
        print_cpu_state();
      #endif
      break;
    case Load: // FALLTHROUGH
    case LoadDone:
      #if TRACE_LOAD
        print_cpu_state();
      #endif  
      break;
    case EmuEnter:
      #if TRACE_EMU_ENTER
        print_cpu_state();
      #endif
      break;
    case EmuExit:
      #if TRACE_EMU_EXIT
        print_cpu_state();
      #endif
      break;
    case Execute:
      #if TRACE_EXECUTE
        print_cpu_state();
      #endif 
      break;
    case ExecuteDone: // FALLTHROUGH
    case ExecuteFinalize:
      #if TRACE_FINALIZE
        print_cpu_state();
      #endif
      break;
    case Store:
      #if TRACE_STORE
        print_cpu_state();  
      #endif
      break;
  }

  // Transition to next T-state.
  switch(CPU.bus_cycle) {
    case T1:
      // Begin a bus cycle only if signalled, otherwise wait in T1
      if(CPU.bus_state != PASV) {
        CPU.bus_cycle = T2;
      }
      break;

    case T2:
      CPU.bus_cycle = T3;
      break;

    case T3:
      // TODO: Handle wait states between t3 & t4
      CPU.bus_cycle = T4;
      break;

    case T4:
      CPU.bus_cycle = T1;
      CPU.bus_state_latched = PASV;
      break;
  }
}

void print_addr(unsigned long addr) {
  static char addr_buf[6];
  snprintf(addr_buf, 6, "%05lX", addr);
  Serial1.println(addr_buf);
}

// Detect the CPU type based on the number of CPU cycles spent executing the 
// CPU ID program.
void detect_cpu_type(uint32_t cpuid_cycles) {
  if (CPU.width == BusWidthEight) {
    if (cpuid_cycles > 5) { 
      //Serial1.println("detect_cpu_type(): Detected NEC V20");
      CPU.cpu_type = necV20;
    }
    else {
      //Serial1.println("detect_cpu_type(): Detected i8088");
      CPU.cpu_type = i8088;
    } 
  }
  else {
    if (cpuid_cycles > 5) { 
      //Serial1.println("detect_cpu_type(): Detected NEC V30");
      CPU.cpu_type = necV30;
    }
    else {
      //Serial1.println("detect_cpu_type(): Detected i8086");
      CPU.cpu_type = i8086;
    }
  }
}

// Main sketch loop 
void loop() {
  
  switch(SERVER.c_state) {

    case WaitingForCommand:
      if(SERIAL.available() > 0) {
        uint8_t cmd_byte = SERIAL.read();

        debug_cmd(CMD_STRINGS[cmd_byte], "received!");

        bool got_command = false;
        if(cmd_byte >= (uint8_t)CmdInvalid) {
          // Command is out of range, check against alias list
          for( uint8_t a = 0; a < sizeof CMD_ALIASES; a++) {
            if(cmd_byte == CMD_ALIASES[a]) {
              /*
              SERIAL.print("Got command alias: ");
              SERIAL.println(cmd_byte, HEX);
              */
              cmd_byte = a;
              got_command = true;
              break;
            }
          }

          if(!got_command) {
            send_fail();
            break;
          }
        }
        
        // Valid command, enter ReadingCommand state
        SERVER.cmd = (server_command)cmd_byte;

        if(cmd_byte == 0) {
          // We ignore command byte 0 (null command)
          break;
        }
        else if (cmd_byte > MAX_CMD) {
          // Cmd is out of range
          debug_proto("Command out of range!");
          break;
        }
        else if(CMD_INPUTS[cmd_byte] > 0) {
          // This command requires input bytes before it is executed.
          SERVER.cmd = (server_command)cmd_byte;
          SERVER.cmd_byte_n = 0;
          SERVER.c_state = ReadingCommand;
          SERVER.cmd_bytes_expected = CMD_INPUTS[cmd_byte];
          SERVER.cmd_start_time = millis(); // Get start time for timeout calculation
        }
        else {
          // Command requires no input, execute immediately
          bool result = V_TABLE[cmd_byte - 1]();
          if(result) {  
            debug_proto("Command OK!");
            send_ok();
          }
          else {
            debug_proto("Command FAIL!");
            send_fail();
          }
        }
      }
      break;

    case ReadingCommand:
      // The previously specified command requires paramater bytes, so read them in, or timeout
      if(SERIAL.available() > 0) {
        uint8_t cmd_byte = SERIAL.read();
        
        if(SERVER.cmd_byte_n < MAX_COMMAND_BYTES) {
          // Stil have bytes yet to read
          COMMAND_BUFFER[SERVER.cmd_byte_n] = cmd_byte;
          SERVER.cmd_byte_n++;
 
          if(SERVER.cmd_byte_n == SERVER.cmd_bytes_expected) {
            // We have received enough parameter bytes to execute the in-progress command.
            bool result = V_TABLE[SERVER.cmd - 1]();
            if(result) {  
              send_ok();
            }
            else {
              send_fail();
            }

            // Revert to listening for command
            SERVER.cmd_byte_n = 0;
            SERVER.cmd_bytes_expected = 0;
            SERVER.c_state = WaitingForCommand;
          }
        }
      }
      else {
        // No bytes received yet, so keep track of how long we've been waiting
        uint32_t now = millis();
        uint32_t elapsed = now - SERVER.cmd_start_time;

        if(elapsed >= CMD_TIMEOUT) {
          // Timed out waiting for parameter bytes. Send failure and revert to listening for command
          SERVER.cmd_byte_n = 0;
          SERVER.cmd_bytes_expected = 0;          
          SERVER.c_state = WaitingForCommand;
          debug_proto("Command timeout!");
          send_fail();
        }
      }
      break;
  }
}

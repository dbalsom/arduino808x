/*
    ArduinoX86 Copyright 2022-2025 Daniel Balsom
    https://github.com/dbalsom/arduinoX86

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
#include "arduinoX86.h"

uint32_t calc_flat_address(uint16_t seg, uint16_t offset) {
  return ((uint32_t)seg << 4) + offset;
}

uint8_t reverse_byte(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

// -------------------------- CPU Interface -----------------------------------

// Execute one clock pulse to the CPU
void clock_tick() {
  WRITE_CLK(1);
  delayMicroseconds(CLOCK_PIN_HIGH_DELAY);
  WRITE_CLK(0);
  // 186 input clock is 2x output (CPU) clock
  #if CPU_186
    delayMicroseconds(CLOCK_PIN_HIGH_DELAY);
    WRITE_CLK(1);
    delayMicroseconds(CLOCK_PIN_HIGH_DELAY);
    WRITE_CLK(0);
  #endif
  delayMicroseconds(CLOCK_PIN_LOW_DELAY);
  tick_i8288();
}

// Read the status lines S0-S5 as well as queue status lines QS0-QS1.
uint8_t read_status0_raw() {
  
  uint8_t status0 = 0;
  status0 |= READ_PIN_D14 ? 0x01 : 0;     // S0  - Pin 14
  status0 |= READ_PIN_D15 ? 0x02 : 0;     // S1  - Pin 15
  status0 |= READ_PIN_D16 ? 0x04 : 0;     // S2  - Pin 16
  status0 |= READ_PIN_D38 ? 0x08 : 0;     // S3  - Pin 38
  status0 |= READ_PIN_D39 ? 0x10 : 0;     // S4  - Pin 39
  status0 |= READ_PIN_D40 ? 0x20 : 0;     // S5  - Pin 40
  status0 |= READ_PIN_D09 ? 0x40 : 0;     // QS0 - Pin 9
  status0 |= READ_PIN_D08 ? 0x80 : 0;     // QS1 - Pin 8

  return status0;
}

void read_status0() {
  CPU.status0 = read_status0_raw();
}

// Read the i8288 command lines (and BHE pin)
void read_8288_command_bits() {
  uint8_t command = 0;
  command |= READ_MRDC_PIN ? 0x01 : 0;     // MRDC - Pin 51
  command |= READ_AMWC_PIN ? 0x02 : 0;     // AMWC - Pin 52
  command |= READ_MWTC_PIN ? 0x04 : 0;     // MWTC - Pin 53
  command |= READ_IORC_PIN ? 0x08 : 0;     // IORC - Pin 46
  command |= READ_AIOWC_PIN ? 0x10 : 0;    // AIOWC- Pin 48
  command |= READ_IOWC_PIN ? 0x20 : 0;     // IOWC - Pin 47
  command |= READ_INTA_PIN ? 0x40 : 0;     // INTA - Pin 45
  // Although not an 8288 command status, we have an extra bit, so we can stick BHE in here.
  // This saves us from needing to add an extra byte - that adds up!
  command |= READ_BHE_PIN ? 0x80 : 0;      // BHE  - Pin 17
  CPU.command_bits = command;
}

// Read the i8288 control lines
void read_8288_control_bits() {
  uint8_t control = 0;
  control |= READ_ALE_PIN ? 0x01 : 0;     // ALE      - Pin 50
  control |= READ_PIN_D49 ? 0x02 : 0;     // DTR      - Pin 49
  control |= READ_PIN_D43 ? 0x04 : 0;     // MCE/PDEN - Pin 43
  control |= READ_PIN_D44 ? 0x08 : 0;     // DEN      - Pin 44
  CPU.control_bits = control;
}

// Resets the CPU by asserting RESET line for a period. 
// Successful RESET should be indicated by an ALE signalling start of the first CODE fetch.
bool cpu_reset() {

  static char buf[7];

  digitalWrite(TEST_PIN, LOW);
  digitalWrite(INTR_PIN, LOW); 
  digitalWrite(NMI_PIN, LOW);

  #if CPU_186
    // Set A16-A19 to output
    pinMode(38, OUTPUT);
    pinMode(39, OUTPUT);
    pinMode(40, OUTPUT);
    pinMode(41, OUTPUT);
    // Drive A16-A19 high during RESET. This avoids entering ONCE test mode
    digitalWrite(38, HIGH);
    digitalWrite(39, HIGH);
    digitalWrite(40, HIGH);
    digitalWrite(41, HIGH);
  #endif

  reset_cpu_struct(false);

  //CYCLE_NUM_H = 0;
  CYCLE_NUM = 0;
  bool ale_went_off = false;
  bool bhe_went_off = false;
  bool qs0_high = false;

  // Assert RESET high for hold count.
  WRITE_RESET(RESET_ASSERT);

  for (int i = 0; i < RESET_HOLD_CYCLE_COUNT; i++) {
    if (READ_ALE_PIN == false) {
      ale_went_off = true;
    }
    cycle();
  }

  // For < 186, ALE should have gone off during RESET hold. 
  // For 186, it happens a bit after.
  #if !CPU_186
    // CPU didn't reset for some reason.
    if (ale_went_off == false) {
      set_error("CPU failed to reset: ALE not off!");   
      return false;
    }
  #endif

  WRITE_RESET(RESET_DEASSERT);

  // Clock CPU while waiting for ALE
  int ale_cycles = 0;

  // Reset takes 7 cycles, bit we can try for longer
  for ( int i = 0; i < RESET_CYCLE_TIMEOUT; i++ ) {
    cycle();

    if (READ_QS0_PIN) {
      qs0_high = true;
    }

    if (!READ_ALE_PIN) {
      if (!ale_went_off) {
        ale_went_off = true;
      }
    }

    if (!READ_BHE_PIN) {
        bhe_went_off = true;
    }

    read_status0();
    #if MODE_ASCII  
      snprintf(buf, 3, "%01X", CPU.status0 & 0x07);
      SERIAL.print(buf);
    #endif
    //clock_tick();
    ale_cycles++;      

    if (ale_went_off && READ_ALE_PIN) {
      // ALE is active! CPU has successfully reset
      CPU.doing_reset = false;
      #if DEBUG_RESET
          debugPrintlnColor(ansi::green, "###########################################");
        if (bhe_went_off) {
          debugPrintlnColor(ansi::green, "## Reset CPU: 16-bit bus detected        ##");
        }
        else {
          debugPrintlnColor(ansi::green, "## Reset CPU:  8-bit bus detected        ##");
        }
        if (qs0_high) {
          debugPrintlnColor(ansi::green, "## Queue status lines appear unavailable ##");
        }
          debugPrintlnColor(ansi::green, "###########################################");
      #endif

      if (bhe_went_off) {
        cpu_set_width(BusWidthSixteen);
      }
      else {
        cpu_set_width(BusWidthEight);
      }

      CPU.have_queue_status = !qs0_high;

      #if CPU_186
        // Return A16-A19 pins to INPUT.
        pinMode(38, INPUT);
        pinMode(39, INPUT);
        pinMode(40, INPUT);
        pinMode(41, INPUT);
      #endif

      return true;
    }
  }

  // ALE did not turn on within the specified cycle timeout, so we failed to reset the cpu.
  #if DEBUG_RESET
    Serial1.println("## Failed to reset CPU! ##");
  #endif
  set_error("CPU failed to reset: No ALE!");   


  #if CPU_186
    // Don't leave the reset pin high - we can try removing power to the CPU if it fails
    // to reset properly.  Reset must be asserted (low) when CPU receives power.
    WRITE_RESET(RESET_ASSERT);
  #endif

  return false;
}

void cpu_set_width(cpu_width_t width) {
  CPU.width = width;

  if (width == BusWidthEight) {
    CPU.queue.size = 4;
  }
  else {
    CPU.queue.size = 6;
  }
}

// ----------------------------------Opcodes-----------------------------------


const char *get_opcode_str(uint8_t op1, uint8_t op2, bool modrm) {
  if (CPU.in_emulation) {
    return get_80_opcode_str(op1, op2);
  }
  else {
    return get_86_opcode_str(op1, op2, modrm);
  }
}

const char *get_80_opcode_str(uint8_t op1, uint8_t op2) {
  size_t op_idx = (size_t)OPCODE_8080_REFS[op1];

  if (op1 == 0xED) {
    if (op2 == 0xEF) {
      return "CALLN";
    }
    else if (op2 == 0xFD) {
      return "RETEM";
    }
    else {
      return "INVAL";
    }
  }

  return OPCODE_8080_STRS[(size_t)op_idx];
}

// Return the mnemonic name for the specified opcode. If the opcode is a group
// opcode, op2 should be specified and modrm set to true.
const char *get_86_opcode_str(uint8_t op1, uint8_t op2, bool modrm) {

  size_t op_idx = (size_t)OPCODE_REFS[op1];
  size_t grp_idx = 0;

  if(!modrm) {
    // Just return primary opcode
    return OPCODE_STRS[op_idx];
  }
  else {
    // modrm is in use, check if this is a group instruction...
    if(IS_GRP_OP(op1)) {  
      // Lookup opcode group
      grp_idx = MODRM_OP(op2);

      switch(OPCODE_REFS[op1]) {
        case GRP1:
          return OPCODE_STRS_GRP1[grp_idx];
          break;        
        case GRP2A:
          return OPCODE_STRS_GRP2A[grp_idx];        
          break;    
        case GRP2B:
          return OPCODE_STRS_GRP2B[grp_idx];         
          break;                   
        case GRP3:
          return OPCODE_STRS_GRP3[grp_idx];        
          break;        
        case GRP4:
          return OPCODE_STRS_GRP4[grp_idx];          
          break;        
        case GRP5:
          return OPCODE_STRS_GRP5[grp_idx];         
          break;
        default:
          return "***";
          break;
      }
    }
    else {
      // Not a group instruction, just return as normal
      return OPCODE_STRS[op_idx];
    }
  }
}
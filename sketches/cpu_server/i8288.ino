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

void tick_i8288() {
  #if EMULATE_8288
    I8288.last_status = (s_state)I8288.status;
    I8288.status = (s_state)(read_status0_raw() & 0x07);

    // TODO: Handle wait states
    switch(I8288.tcycle) {
      case TI:
        break;
      case T1:
        I8288.ale = false;
        I8288.tcycle = T2;
        switch(I8288.status_latch) {
            case IOR:
              I8288.iorc = true;
              break;
            case IOW:
              // Set AIOWC line on T3, IOWC is delayed to T3
              I8288.aiowc = true;
              break;
            case MEMW:
              // Set AMWC line on T2, MWTC is delayed to T3
              I8288.amwc = true;
              break;
            case CODE:
              I8288.mrdc = true;
              break;          
            case MEMR:
              I8288.mrdc = true;
              break;                  
        }
        break;
      case T2:
        I8288.tcycle = T3;
        switch(I8288.status_latch) {
            case IRQA:
              break;
            case IOW:
              I8288.iowc = true;
              break;
            case MEMW:
              I8288.mwtc = true;
              break;
        }        
        break;
      case T3:
        I8288.tcycle = T4;
        I8288.iorc = false;
        I8288.amwc = false;
        I8288.iowc = false;
        I8288.mrdc = false;
        I8288.aiowc = false;
        I8288.mwtc = false;        
      case T4:
        I8288.tcycle = TI;
        break;        
    }

    if (I8288.last_status == PASV && I8288.status != PASV) {
      // We started a bus cycle; enter t1 and set ALE
      I8288.ale = true;
      I8288.tcycle = T1;
      I8288.status_latch = I8288.status;
    }

  #else
    // Nothing to tick.
  #endif
}

void reset_i8288() {
  #if EMULATE_8288
    I8288.last_status = PASV;
    I8288.status = PASV;
    I8288.status_latch = PASV;
    I8288.tcycle = TI;
    I8288.ale = false;
    I8288.mrdc = false;
    I8288.amwc = false;
    I8288.mwtc = false;
    I8288.iorc = false;
    I8288.aiowc = false;
    I8288.iowc = false;
    I8288.inta = false;
  #endif
}
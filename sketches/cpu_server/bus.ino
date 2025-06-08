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

// Functions for reading and writing the CPU bus.


// Write a value to the CPU's data bus
void data_bus_write(uint16_t data, data_width_t width) {

  #if defined(__SAM3X8E__) // If Arduino DUE

    if ((width == EightLow) || (width == Sixteen)) {
      // Set data bus pins to OUTPUT
      PIOB->PIO_OER = BIT26;      // Pin 22
      PIOA->PIO_OER = BIT14 | BIT15; // Pins 23, 24
      PIOD->PIO_OER = BIT00 | BIT01 | BIT02 | BIT03 | BIT06; // Pins 25-29 except 28
    }

    if ((width == EightHigh) || (width == Sixteen)) {
      // Set pins to OUTPUT
      PIOD->PIO_OER = BIT09 | BIT10; // Pins 30 & 32
      PIOA->PIO_OER = BIT07; // Pin 31
      PIOC->PIO_OER = BIT01 | BIT02 | BIT03 | BIT04 | BIT05; // Pins 33-37
    }
    delayMicroseconds(PIN_CHANGE_DELAY);

    if ((width == EightLow) || (width == Sixteen)) {
      // Write low-order byte to data bus pins
      (data & 0x01) ? PIOB->PIO_SODR = BIT26 : PIOB->PIO_CODR = BIT26;      // Pin 22
      (data & 0x02) ? PIOA->PIO_SODR = BIT14 : PIOA->PIO_CODR = BIT14;      // Pin 23
      (data & 0x04) ? PIOA->PIO_SODR = BIT15 : PIOA->PIO_CODR = BIT15;      // Pin 24
      (data & 0x08) ? PIOD->PIO_SODR = BIT00 : PIOD->PIO_CODR = BIT00;      // Pin 25
      (data & 0x10) ? PIOD->PIO_SODR = BIT01 : PIOD->PIO_CODR = BIT01;      // Pin 26
      (data & 0x20) ? PIOD->PIO_SODR = BIT02 : PIOD->PIO_CODR = BIT02;      // Pin 27
      (data & 0x40) ? PIOD->PIO_SODR = BIT03 : PIOD->PIO_CODR = BIT03;      // Pin 28
      (data & 0x80) ? PIOD->PIO_SODR = BIT06 : PIOD->PIO_CODR = BIT06;      // Pin 29
    }

    if ((width == EightHigh) || (width == Sixteen)) {
      (data & 0x0100) ? PIOD->PIO_SODR = BIT09 : PIOD->PIO_CODR = BIT09;    // AD8 Pin 30 (PD9)
      (data & 0x0200) ? PIOA->PIO_SODR = BIT07 : PIOA->PIO_CODR = BIT07;    // AD9 Pin 31 (PA7)
      (data & 0x0400) ? PIOD->PIO_SODR = BIT10 : PIOD->PIO_CODR = BIT10;    // AD10 Pin 32 (PD10)
      (data & 0x0800) ? PIOC->PIO_SODR = BIT01 : PIOC->PIO_CODR = BIT01;    // AD11 Pin 33 (PC1)
      (data & 0x1000) ? PIOC->PIO_SODR = BIT02 : PIOC->PIO_CODR = BIT02;    // AD12 Pin 34 (PC2)
      (data & 0x2000) ? PIOC->PIO_SODR = BIT03 : PIOC->PIO_CODR = BIT03;    // AD13 Pin 35 (PC3)
      (data & 0x4000) ? PIOC->PIO_SODR = BIT04 : PIOC->PIO_CODR = BIT04;    // AD14 Pin 36 (PC4)
      (data & 0x8000) ? PIOC->PIO_SODR = BIT05 : PIOC->PIO_CODR = BIT05;    // AD15 Pin 37 (PC5)
    }

  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA  
    // Set data bus pins 22-29 to OUTPUT
    DDRA = 0xFF;
    delayMicroseconds(PIN_CHANGE_DELAY);
    // TODO: Support 8086
    // Write byte to data bus pins 22-29
    PORTA = data;
  #endif
}

// Read a value from the CPU's data bus
uint16_t data_bus_read(data_width_t width) {

  #if defined(__SAM3X8E__) // If Arduino DUE  
    uint16_t data = 0;

    if ((width == EightLow) || (width == Sixteen)) {
      // Set data bus pins to INPUT
      PIOB->PIO_ODR = BIT26;      // Pin 22
      PIOA->PIO_ODR = BIT14 | BIT15; // Pins 23, 24
      PIOD->PIO_ODR = BIT00 | BIT01 | BIT02 | BIT03 | BIT06; // Pins 25-29 except 28
    }
    if ((width == EightHigh) || (width == Sixteen)) {
      // Set pins to INPUT
      PIOD->PIO_ODR = BIT09 | BIT10; // Pins 30 & 32
      PIOA->PIO_ODR = BIT07; // Pin 31
      PIOC->PIO_ODR = BIT01 | BIT02 | BIT03 | BIT04 | BIT05; // Pins 33-37
    }
    delayMicroseconds(PIN_CHANGE_DELAY);

    if ((width == EightLow) || (width == Sixteen)) {
      // Read data from bus pins
      data |= (PIOB->PIO_PDSR & BIT26) ? 0x01 : 0x00;     // Pin 22, Bit 0 of byte
      data |= (PIOA->PIO_PDSR & BIT14) ? 0x02 : 0x00;     // Pin 23, Bit 1 of byte
      data |= (PIOA->PIO_PDSR & BIT15) ? 0x04 : 0x00;     // Pin 24, Bit 2 of byte
      data |= (PIOD->PIO_PDSR & BIT00) ? 0x08 : 0x00;     // Pin 25, Bit 3 of byte
      data |= (PIOD->PIO_PDSR & BIT01) ? 0x10 : 0x00;     // Pin 26, Bit 4 of byte
      data |= (PIOD->PIO_PDSR & BIT02) ? 0x20 : 0x00;     // Pin 27, Bit 5 of byte
      data |= (PIOD->PIO_PDSR & BIT03) ? 0x40 : 0x00;     // Pin 28, Bit 6 of byte
      data |= (PIOD->PIO_PDSR & BIT06) ? 0x80 : 0x00;     // Pin 29, Bit 7 of byte
    }

    if ((width == EightHigh) || (width == Sixteen)) {
      data |= PIOD->PIO_PDSR & BIT09 ? 0x0100 : 0x0000;   // AD8 Pin 30 (PD9)
      data |= PIOA->PIO_PDSR & BIT07 ? 0x0200 : 0x0000;   // AD9 Pin 31 (PA7)
      data |= PIOD->PIO_PDSR & BIT10 ? 0x0400 : 0x0000;   // AD10 Pin 32 (PD10)
      data |= PIOC->PIO_PDSR & BIT01 ? 0x0800 : 0x0000;   // AD11 Pin 33 (PC1)
      data |= PIOC->PIO_PDSR & BIT02 ? 0x1000 : 0x0000;   // AD12 Pin 34 (PC2)
      data |= PIOC->PIO_PDSR & BIT03 ? 0x2000 : 0x0000;   // AD13 Pin 35 (PC3)
      data |= PIOC->PIO_PDSR & BIT04 ? 0x4000 : 0x0000;   // AD14 Pin 36 (PC4)
      data |= PIOC->PIO_PDSR & BIT05 ? 0x8000 : 0x0000;   // AD15 Pin 37 (PC5)
    }
    return data;

  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA  
    // Set data bus pins 22-29 to INPUT
    DDRA = 0;
    delayMicroseconds(PIN_CHANGE_DELAY);
    // Read LO byte from data bus pins 22-29
    data = PINA;
    
    if (width == Sixteen) {
      // Set data bus pins 30-37 to INPUT
      DDRC = 0;
      delayMicroseconds(PIN_CHANGE_DELAY);
      // Read HO byte from data bus pins 30-37. These are in reversed order for some reason.
      data |= ((uint16_t)reverse_byte(PINC) << 8);
    }

    return data;

  #endif
}

// Read what value is being output on the data bus
uint16_t data_bus_peek(cpu_width_t width) {

  #if defined(__SAM3X8E__) // If Arduino DUE  
    uint16_t data = 0;

    if ((width == EightLow) || (width == Sixteen)) {
      // Read data from bus pins
      data |= (PIOB->PIO_ODSR & BIT26) ? 0x01 : 0x00;     // Pin 22, Bit 0 of byte
      data |= (PIOA->PIO_ODSR & BIT14) ? 0x02 : 0x00;     // Pin 23, Bit 1 of byte
      data |= (PIOA->PIO_ODSR & BIT15) ? 0x04 : 0x00;     // Pin 24, Bit 2 of byte
      data |= (PIOD->PIO_ODSR & BIT00) ? 0x08 : 0x00;     // Pin 25, Bit 3 of byte
      data |= (PIOD->PIO_ODSR & BIT01) ? 0x10 : 0x00;     // Pin 26, Bit 4 of byte
      data |= (PIOD->PIO_ODSR & BIT02) ? 0x20 : 0x00;     // Pin 27, Bit 5 of byte
      data |= (PIOD->PIO_ODSR & BIT03) ? 0x40 : 0x00;     // Pin 28, Bit 6 of byte
      data |= (PIOD->PIO_ODSR & BIT06) ? 0x80 : 0x00;     // Pin 29, Bit 7 of byte
    }

    if ((width == EightHigh) || (width == Sixteen)) {
      data |= PIOD->PIO_ODSR & BIT09 ? 0x0100 : 0x0000;   // AD8 Pin 30 (PD9)
      data |= PIOA->PIO_ODSR & BIT07 ? 0x0200 : 0x0000;   // AD9 Pin 31 (PA7)
      data |= PIOD->PIO_ODSR & BIT10 ? 0x0400 : 0x0000;   // AD10 Pin 32 (PD10)
      data |= PIOC->PIO_ODSR & BIT01 ? 0x0800 : 0x0000;   // AD11 Pin 33 (PC1)
      data |= PIOC->PIO_ODSR & BIT02 ? 0x1000 : 0x0000;   // AD12 Pin 34 (PC2)
      data |= PIOC->PIO_ODSR & BIT03 ? 0x2000 : 0x0000;   // AD13 Pin 35 (PC3)
      data |= PIOC->PIO_ODSR & BIT04 ? 0x4000 : 0x0000;   // AD14 Pin 36 (PC4)
      data |= PIOC->PIO_ODSR & BIT05 ? 0x8000 : 0x0000;   // AD15 Pin 37 (PC5)
    }
    return data;

  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA  
    // Read LO byte from data bus pins 22-29
    data = PINA;
    if (width == Sixteen) {
      // Read HO byte from data bus pins 30-37. These are in reversed order for some reason.
      data |= ((uint16_t)reverse_byte(PINC) << 8);
    }
    return data;
  #endif
}

void read_address() {
  CPU.address_bus = read_address_pins(false);
}

uint32_t peek_address() {
  return read_address_pins(true);
}

void latch_address() {
  uint32_t addr = read_address_pins(false);
  CPU.address_bus = addr;
  CPU.address_latch = addr;
}

// Return the value of address line 0 as a bool representing if the address is odd
bool a0() {
  return (CPU.address_latch & 1) == 1;
}

/*
  Read the address pins and return the 20 bit value in a uint32
  Note: address is only valid while ALE is HIGH (on T1) Otherwise mutiplexed with status and data.
*/
uint32_t read_address_pins(bool peek) {

  uint32_t address = 0;

  #if defined(__SAM3X8E__) // If Arduino DUE  
    
    // If 'peeking' at the bus, we want to see what is being output as well as input. So we don't change
    // pin direction.
    if(!peek) {
      // Set data bus pins to INPUT
      uint32_t pins_b = BIT26;                                    // Pin 22
      uint32_t pins_a = BIT07 | BIT14 | BIT15;                    // Pins 23, 24, 31
      uint32_t pins_c = 0x01FF;                                   // Pins 33-41
      uint32_t pins_d = BIT00 | BIT01 | BIT02 | BIT03 | BIT06 | BIT09 | BIT10;    // Pins 25-32 except 31  
      
      PIOA->PIO_ODR = pins_a;
      PIOB->PIO_ODR = pins_b;
      PIOC->PIO_ODR = pins_c;
      PIOD->PIO_ODR = pins_d;
      delayMicroseconds(PIN_CHANGE_DELAY); // Wait for pin state change before reading
    }

    address |= (PIOB->PIO_PDSR & BIT26) ? 0x00000001 : 0;     // AD0  Pin 22 (PB26)
    address |= (PIOA->PIO_PDSR & BIT14) ? 0x00000002 : 0;     // AD1  Pin 23 (PA14)
    address |= (PIOA->PIO_PDSR & BIT15) ? 0x00000004 : 0;     // AD2  Pin 24 (PA15)
    address |= (PIOD->PIO_PDSR & BIT00) ? 0x00000008 : 0;     // AD3  Pin 25 (PD0)
    address |= (PIOD->PIO_PDSR & BIT01) ? 0x00000010 : 0;     // AD4  Pin 26 (PD1)
    address |= (PIOD->PIO_PDSR & BIT02) ? 0x00000020 : 0;     // AD5  Pin 27 (PD2)
    address |= (PIOD->PIO_PDSR & BIT03) ? 0x00000040 : 0;     // AD6  Pin 28 (PD3)
    address |= (PIOD->PIO_PDSR & BIT06) ? 0x00000080 : 0;     // AD7  Pin 29 (PD6)
    address |= (PIOD->PIO_PDSR & BIT09) ? 0x00000100 : 0;     // AD8  Pin 30 (PD9)
    address |= (PIOA->PIO_PDSR & BIT07) ? 0x00000200 : 0;     // AD9  Pin 31 (PA7)
    address |= (PIOD->PIO_PDSR & BIT10) ? 0x00000400 : 0;     // AD10 Pin 32 (PD10)

    address |= (PIOC->PIO_PDSR & BIT01) ? 0x00000800 : 0;     // AD11 Pin 33
    address |= (PIOC->PIO_PDSR & BIT02) ? 0x00001000 : 0;     // AD12 Pin 34
    address |= (PIOC->PIO_PDSR & BIT03) ? 0x00002000 : 0;     // AD13 Pin 35
    address |= (PIOC->PIO_PDSR & BIT04) ? 0x00004000 : 0;     // AD14 Pin 36
    address |= (PIOC->PIO_PDSR & BIT05) ? 0x00008000 : 0;     // AD15 Pin 37
    address |= (PIOC->PIO_PDSR & BIT06) ? 0x00010000 : 0;     // AD16 Pin 38
    address |= (PIOC->PIO_PDSR & BIT07) ? 0x00020000 : 0;     // AD17 Pin 39
    address |= (PIOC->PIO_PDSR & BIT08) ? 0x00040000 : 0;     // AD18 Pin 40
    address |= (PIOC->PIO_PDSR & BIT09) ? 0x00080000 : 0;     // AD19 Pin 41
  #elif defined (ARDUINO_GIGA)
    address |= READ_PIN_D22 ? 0x00000001 : 0;     // AD0  Pin 22
    address |= READ_PIN_D23 ? 0x00000002 : 0;     // AD1  Pin 23
    address |= READ_PIN_D24 ? 0x00000004 : 0;     // AD2  Pin 24
    address |= READ_PIN_D25 ? 0x00000008 : 0;     // AD3  Pin 25
    address |= READ_PIN_D26 ? 0x00000010 : 0;     // AD4  Pin 26
    address |= READ_PIN_D27 ? 0x00000020 : 0;     // AD5  Pin 27
    address |= READ_PIN_D28 ? 0x00000040 : 0;     // AD6  Pin 28
    address |= READ_PIN_D29 ? 0x00000080 : 0;     // AD7  Pin 29
    address |= READ_PIN_D30 ? 0x00000100 : 0;     // AD8  Pin 30
    address |= READ_PIN_D31 ? 0x00000200 : 0;     // AD9  Pin 31
    address |= READ_PIN_D32 ? 0x00000400 : 0;     // AD10 Pin 32
    address |= READ_PIN_D33 ? 0x00000800 : 0;     // AD11 Pin 33
    address |= READ_PIN_D34 ? 0x00001000 : 0;     // AD12 Pin 34
    address |= READ_PIN_D35 ? 0x00002000 : 0;     // AD13 Pin 35
    address |= READ_PIN_D36 ? 0x00004000 : 0;     // AD14 Pin 36
    address |= READ_PIN_D37 ? 0x00008000 : 0;     // AD15 Pin 37
    address |= READ_PIN_D38 ? 0x00010000 : 0;     // AD16 Pin 38
    address |= READ_PIN_D39 ? 0x00020000 : 0;     // AD17 Pin 39
    address |= READ_PIN_D40 ? 0x00040000 : 0;     // AD18 Pin 40
    address |= READ_PIN_D41 ? 0x00080000 : 0;     // AD19 Pin 41
  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA 
    // Set data bus pins 22-29 to INPUT
    if (!peek) {
      DDRA = 0;
      delayMicroseconds(PIN_CHANGE_DELAY); // Wait for pin state change before reading
    }
    address = PINA; // Pins 22-29
    address |= (unsigned long)BIT_REVERSE_TABLE[PINC] << 8; // Pins 30-37 (Bit order reversed)
    address |= (unsigned long)(PIND & 0x80) << 9; // Pin 38
    address |= (unsigned long)(BIT_REVERSE_TABLE[PING] & 0xE0) << 12; // Pins 39-40 (Bit order reversed)  
  #endif

  return address;
}

# Arduino808X
![arduino8088_pcb](/images/render_v1_1.png)

### About Arduino808X
I've written a blog article that gives an overview of this project and how it is used.

https://martypc.blogspot.com/2023/06/hardware-validating-emulator.html

### Description

This project expands on the basic idea of controlling an Intel 8088 or NEC V20 CPU via GPIO pins to clock the CPU and read and write control and data signals.
This can be used to validate an emulator's accuracy, but also as a general method of exploring the operation of 8088 and V20 instructions and timings.

Where it differs from Raspberry Pi based projects is that it uses an Arduino DUE to supply enough GPIO pins to operate the 8088 in Maximum mode without requiring any shifters. This enables several useful signals to be read such as the QS0 & QS1 processor instruction queue status lines, which give us more insight into the internal state of the CPU. We can also enable inputs such as READY, NMI, INTR, and TEST, so we can in theory execute interrupts, emulate wait states, and perhaps simulate FPU and DMA operations.

The board supports an Intel 8288 bus controller which can produce bus signals, but this chip is optional as the sketch can perform i8288 emulation.

The original Arduino808X board utilized an Arduino MEGA, but this board is now considered deprecated for this project. As of the current release, an [Arduino DUE](https://store.arduino.cc/products/arduino-due) should be used instead. Although the DUE has 3v GPIO, the current board design is modified for 3V operation.  The 80C88 itself tolerates 3V well, and i8288 emulation can be used if you lack a CMOS 8288.

I have been using this project to validate the cycle-accuracy of my PC emulator, MartyPC: https://github.com/dbalsom/martypc 

## Can the CPU be clocked fast enough?

In short, no. We are well past the published minimum cycle times when executing programs via a serial protocol, cycle by cycle. Some chips tolerate this better than others. When working with an Intel branded 8088, I noticed that effective address calculations were failing to add the displacement or index register, but otherwise functioned. I have had more luck with the AMD second-source 8088 CPUs, which seem to function perfectly under slow clocks although they will hang and need to be reset if not cycled for a several milliseconds. The issue is "dynamic logic" - logic gates that lose their state if not refrehsed electrically within a frequent enough interval. To be absolutely safe, it is best to use a fully CMOS process CPU such as the 80C88. 

## Credits

Inspired by and borrows from the Pi8088 validator created by Andreas Jonsson as part of the VirtualXT project:

https://github.com/andreas-jonsson/virtualxt/tree/develop/tools/validator/pi8088

A very similar project is homebrew8088's Raspberry Pi Hat:

https://github.com/homebrew8088/pi86

## To use

If you don't want to order and build the PCB, connect the GPIO pins to the CPU on a breadboard as specified in the KiCad project schematic.

The main Arduino808X sketch, cpu_server, operates a simple binary serial protocol to execute operations on the CPU and read and write data, status and control signals. This is designed for integration with an emulator or instruction test generator.

Additionally, there is a sketch, 'run_program', that will take any user-supplied register state and array of instruction bytes defined in the source code, execute it, and print cycle traces and final register state. This is useful for investigating the timing and operation of certain instructions without needing any external software integrations, however it is restricted in the number of memory reads or writes it can support, due to the limited RAM on the Arduino MEGA (8k!) 'run_program' does not currently support the DUE.

An example application for cpu_server is provided, written in Rust, in the /crates/exec_program directory. It demonstrates how to upload arbitrary code to the Arduino808X and display cycle traces. The client will emulate the entire address space and set up a basic IVT.

## PCB
![pcb_shield50](/images/pcb_v1_1.png)

KiCad project files for the PCB are supplied. 

In theory the board could also support an 8086 CPU. Version 1.1 adds a connection for the 8086's BHE pin, 
which indicates the size of the current bus transfer.  The cpu_server sketch and protocol still needs modification to support 16 bit data transfers and the longer queue length on the 8086.

Please read all the notes in the next section before ordering/assembling parts. Failure to heed warnings will cause damage to your Arduino.

# BOM
- A compatible CPU. For best results, use a CMOS CPU such as a Harris 80C88, Oki 80C88, or NEC V20 CPU. Beware of counterfeits on eBay and other online vendors.
A legitimate chip will not look shiny and new with perfect printing on it.

- (Optional) An Intel 8288 or OKI 82C88 Bus Controller. If not using an 8288, set the EMULATE_8288 flag in cpu_server.

- A set of Arduino stacking headers (also usable with DUE) 
https://www.amazon.com/Treedix-Stacking-Headers-Stackable-Compatible/dp/B08G4FGBPQ

- A DIP-40 and (optionally) DIP-20 socket
  - Optional: You can spring for a ZIF socket such as [https://www.amazon.com/-/en/gp/product/B00B886OZI](https://www.amazon.com/-/en/gp/product/B00B886OZI)

- (2x) 0805 0.047uf bypass capacitors
  https://www.mouser.com/ProductDetail/80-C0805C473KARAUTO

- (Optional) A 12mm, active buzzer with 7.6mm pin spacing. 

  - For DUE: A 3V piezoelectric, low power buzzer <= 6mA
    https://www.mouser.com/ProductDetail/Mallory-Sonalert/PK-11N40PQ?qs=SXHtpsd1MbZ%252B7jeUyAAOVA%3D%3D
    
  - For MEGA: Any 3-5V buzzer <= 30mA
    WARNING: Only connect an electromagnetic buzzer if using an Arduino MEGA.  The DUE has much lower GPIO max current supply.    

- (2x) 750Ohm resistors (for LEDs)
  https://www.mouser.com/ProductDetail/667-ERA-6AED751V

- (2x) Any 0805 ~2V LED of your choice with 1.8-1.9mA forward current
  - https://www.mouser.com/ProductDetail/604-APTD2012LCGCK (Green)
  - https://www.mouser.com/ProductDetail/604-APT2012LSECKJ4RV (Orange)
 
- RS232 board for debug output - choose gender based on your desired cabling
  - https://www.amazon.com/Ultra-Compact-RS232-Converter-1Mbps/dp/B074BMLM11 (male)
  - https://www.amazon.com/Ultra-Compact-RS232-Converter-Female/dp/B074BTGLJN (female)
  - WARNING: DO NOT connect 5V to rs232 board on DUE 

# Project Structure

## /asm

Assembly language files, intended to be assembled with NASM. To execute code on the Arduino808X, one must supply two
binary files, one containing the program to be executed, and one containing the register values to load onto the CPU
before program execution. 

## /crates/ard808x_client

A library crate that implements a client for the Arduino808X's serial protocol.

## /crates/ard808x_cpu

A library crate built on top of the `ard808x_client` crate, this provides a `RemoteCpu` struct that models CPU state 
and can execute programs.

## /crates/exec_program

A binary implementing an interface for the `ard808x_cpu` crate that will load a provided register state binary and 
execute the specified program binary.

## /pcb

Contains the KiCad project files and Gerber files for the Arduino808X PCB.

## /sketches/cpu_server

The main Arduino sketch for Arduino808X. Implements a server for a serial protocol enabling remote control of a 16-bit
Intel CPU on the Arduino DUE.

## /sketches/run_program

An older sketch that can execute a program directly on the Arduino MEGA. Supports the 8088 only.

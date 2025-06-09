; regs.asm
; Compile with nasm to build regs.bin for cpu_client
; nasm regs.asm -o regs.bin

cpu	8086
org	0h

; Specify the initial register state by modifying the values below.
; Assembling this file creates a BIN file representing the initial register state.
; Do not modify the order of the registers or add extra data.
dw 0x1111 ; AX
dw 0x2222 ; BX
dw 0x3333 ; CX
dw 0x4444 ; DX
dw 0x0100 ; IP
dw 0xF000 ; CS
dw 0xF002 ; FLAGS
dw 0x5500 ; SS
dw 0xFFFE ; SP
dw 0x6666 ; DS
dw 0x7777 ; ES
dw 0x8888 ; BP
dw 0xDEAD ; SI
dw 0xBEEF ; DI

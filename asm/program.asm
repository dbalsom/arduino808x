; program.asm
; Compile with nasm to build program.bin for cpu_client
; nasm program.asm -o program.bin
cpu	8086
org	0h

    sti
    mov al, 0x08
    out 0x30, al
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
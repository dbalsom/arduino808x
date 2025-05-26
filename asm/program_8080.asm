; program_8080.asm
; Compile with nasm to build program.bin for cpu_client
; nasm program_8080.asm -o program.bin
%include 'asm/i8080.inc'

cpu	8086
org	100h
code8080

    mvi a, 0x00
    mvi b, 0x00
    add b

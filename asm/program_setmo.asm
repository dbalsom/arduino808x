; program.asm
; Compile with nasm to build program.bin for cpu_client
; nasm program.asm -o program.bin
cpu	8086
org	0h

    db 0D0h
    db 034h

; store.asm
; Original routine by Andreas Jonsson
; https://github.com/andreas-jonsson/virtualxt/tree/develop/tools/validator/pi8088
;
; Assemble with nasm: 
; nasm store.asm -o store.bin

; Registers are output in turn to dummy IO addresses, intercepted by the validator 
; program. End of the routine is indicated by a write to IO address 0xFD.

; This routine is intended to run out of an NMI handler that terminates program
; execution. Therefore IP, CS and FLAGS can be popped from the stack.

cpu	8086
org	0h

    times 6 nop

    out     0xFE, ax        ; AX
    mov     ax, bx
    out     0xFE, ax        ; BX
    mov     ax, cx
    out     0xFE, ax        ; CX
    mov     ax, dx
    out     0xFE, ax        ; DX

    pop     ax              ; Pop IP from the stack.
    out     0xFE, ax        ; IP

    pop     ax              ; Pop CS from the stack.
    out     0xFE, ax        ; CS

    pop     ax              ; Pop Flags from the stack.
    out     0xFE, ax        ; Flags

    mov     ax, ss
    out     0xFE, ax        ; SS
    mov     ax, sp
    out     0xFE, ax        ; SP

    mov     ax, ds
    out     0xFE, ax        ; DS
    mov     ax, es
    out     0xFE, ax        ; ES
    mov     ax, bp
    out     0xFE, ax        ; BP
    mov     ax, si
    out     0xFE, ax        ; SI
    mov     ax, di
    out     0xFE, ax        ; DI

    mov     al, 0xFF        ; Sent as a signal to the validator program that we are done.
    out     0xFD, al        ; Done!

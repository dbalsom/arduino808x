
; kefrensloop.asm
; kefrens effect main loop from 8088mph

cpu	8086
org	0h

  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop  

; start of unrolled loop procedure
  mov al, [es:di]
  mov ds, bp
  lodsb
  out 0xE0, al   ; what is this port(?)

; 2nd iteration for prefetch modelling
  mov ax,9999
  mov ds,ax
  mov sp,[bx]
  pop di
  mov al,[es:di]
  pop cx
  and ax,cx
  pop cx
  or ax,cx
  stosw
  pop ax
  and ah,[es:di+1]
  pop cx
  or ax,cx
  stosw
  pop ax
  out dx,al
  mov ds,bp
  lodsb
  out 0x60,al
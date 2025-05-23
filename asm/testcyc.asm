; testcyc.asm
; cycle-test from 8088mph

cpu	8086
org	100h

    mov     ax, 1234h
    xor     bx, bx
    mov     cx, bx
    mov     dx, 5678h
    mov     si, bx
    mov     di, bx
    add     ax, 1234h
    add     dx, 1234h
    add     al, 12h
    add     dl, 12h
    add     ds:12D6h, ax
    add     ds:12D6h, dx
    add     ax, ds:12D6h
    add     dx, ds:12D6h
    add     ax, dx
    add     dx, ax
    add     al, dl
    add     dl, al
    push    es
    pop     es
    or      ax, 1234h
    or      dx, 1234h
    or      al, 12h
    or      dl, 12h
    or      ds:12D6h, ax
    or      ds:12D6h, dx
    or      ax, ds:12D6h
    or      dx, ds:12D6h
    or      ax, dx
    or      dx, ax
    or      al, dl
    or      dl, al
    push    cs
    pop     es
;    assume es:seg000
    adc     ax, 1234h
    adc     dx, 1234h
    adc     al, 12h
    adc     dl, 12h
    adc     ds:12D6h, ax
    adc     ds:12D6h, dx
    adc     ax, ds:12D6h
    adc     dx, ds:12D6h
    adc     ax, dx
    adc     dx, ax
    adc     al, dl
    adc     dl, al
    push    ax
    mov     ax, sp
    push    ss
    pop     ss
    mov     sp, ax
    pop     ax
    sbb     ax, 1234h
    sbb     dx, 1234h
    sbb     al, 12h
    sbb     dl, 12h
    sbb     ds:12D6h, ax
    sbb     ds:12D6h, dx
    sbb     ax, ds:12D6h
    sbb     dx, ds:12D6h
    sbb     ax, dx
    sbb     dx, ax
    sbb     al, dl
    sbb     dl, al
    push    ds
    pop     ds
    and     ax, 1234h
    and     dx, 1234h
    and     al, 12h
    and     dl, 12h
    and     ds:12D6h, ax
    and     ds:12D6h, dx
    and     ax, ds:12D6h
    and     dx, ds:12D6h
    and     ax, dx
    and     dx, ax
    and     al, dl
    and     dl, al
    mov     ax, es:[bx]
    daa
    sub     ax, 1234h
    sub     dx, 1234h
    sub     al, 12h
    sub     dl, 12h
    sub     ds:12D6h, ax
    sub     ds:12D6h, dx
    sub     ax, ds:12D6h
    sub     dx, ds:12D6h
    sub     ax, dx
    sub     dx, ax
    sub     al, dl
    sub     dl, al
    mov     ax, cs:[bx]
    das
    xor     ax, 1234h
    xor     dx, 1234h
    xor     al, 12h
    xor     dl, 12h
    xor     ds:12D6h, ax
    xor     ds:12D6h, dx
    xor     ax, ds:12D6h
    xor     dx, ds:12D6h
    xor     ax, dx
    xor     dx, ax
    xor     al, dl
    xor     dl, al
    mov     ax, ss:[bx]
    aaa
    cmp     ax, 1234h
    cmp     dx, 1234h
    cmp     al, 12h
    cmp     dl, 12h
    cmp     ds:12D6h, ax
    cmp     ds:12D6h, dx
    cmp     ax, ds:12D6h
    cmp     dx, ds:12D6h
    cmp     ax, dx
    cmp     dx, ax
    cmp     al, dl
    cmp     dl, al
    db      3Eh
    lodsw
    aas
    inc     ax
    inc     cx
    inc     dx
    inc     bx
    inc     si
    inc     di
    dec     ax
    dec     cx
    dec     dx
    dec     bx
    dec     si
    dec     di
    push    ax
    push    cx
    push    dx
    push    bx
    push    bp
    push    si
    push    di
    pop     di
    pop     si
    pop     bp
    pop     bx
    pop     dx
    pop     cx
    pop     ax
    xor     cx, cx
    dec     cx
    stc
    jb      short loc_10911
    nop
    
                            ; CODE XREF: test_cpu_01+17D↑j
                            ; test_cpu_01+181↓j ...
loc_10911:
    clc
    jb      short loc_10911
    inc     cx
    ;jcxz    short loc_10911
    db 0xE3, 0xFA
    sub     cx, 2
    jmp     short loc_1091E
    ;-------------------------------------------------------------
    
                            ; CODE XREF: test_cpu_01:loc_1091E↓j
loc_1091C:
    inc     cx
    clc
    
                            ; CODE XREF: test_cpu_01+189↑j
loc_1091E:
    jbe     short loc_1091C
    mov     cx, 2
    
                            ; CODE XREF: test_cpu_01+193↓j
loc_10923:
    nop
    loop    loc_10923
    test    ax, 1234h
    test    dx, 1234h
    test    al, 12h
    test    dl, 12h
    test    ds:12D6h, ax
    test    ds:12D6h, dx
    test    ds:12D6h, ax
    test    ds:12D6h, dx
    test    dx, ax
    test    ax, dx
    test    dl, al
    test    al, dl
    lea     ax, ds:12D6h
    mov     es, word [bx+si+1234h]
    ;assume es:nothing
    nop
    xchg    ax, ds:12D6h
    xchg    dx, ds:12D6h
    xchg    ax, ds:12D6h
    xchg    dx, ds:12D6h
    xchg    ax, dx
    xchg    ax, dx
    xchg    dl, al
    xchg    al, dl
    cbw
    push    ds
    pop     es
    mov     di, si
    movsb
    movsw
    movsb
    movsw
    lodsb
    stosb
    lodsw
    stosw
    lodsb
    stosb
    lodsw
    stosw
    cmpsb
    cmpsw
    cmpsb
    cmpsw
    scasb
    scasw
    scasb
    scasw
    mov     al, 12h
    mov     cl, 12h
    mov     dl, 12h
    mov     bl, 12h
    mov     ah, 12h
    mov     ch, 12h
    mov     dh, 12h
    mov     bh, 12h
    mov     ax, 1234h
    mov     cx, 1234h
    mov     dx, 1234h
    mov     bx, 1234h
    mov     si, 1234h
    mov     di, 1234h
    les     bx, ds:1234h
    mov     bx, 0FFFFh
    rol     bl, 1
    rol     byte ds:12DCh, 1
    ror     bl, 1
    ror     byte ds:12DCh, 1
    rcl     bl, 1
    rcl     byte ds:12DCh, 1
    rcr     bl, 1
    rcr     byte ds:12DCh, 1
    shl     bl, 1
    shl     byte ds:12DCh, 1
    shr     bl, 1
    shr     byte ds:12DCh, 1
    shl     bl, 1
    shl     byte ds:12DCh, 1
    sar     bl, 1
    sar     byte ds:12DCh, 1
    rol     bx, 1
    rol     word ds:12D6h, 1
    ror     bx, 1
    ror     word ds:12D6h, 1
    rcl     bx, 1
    rcl     word ds:12D6h, 1
    rcr     bx, 1
    rcr     word ds:12D6h, 1
    shl     bx, 1
    shl     word ds:12D6h, 1
    shr     bx, 1
    shr     word ds:12D6h, 1
    shl     bx, 1
    shl     word ds:12D6h, 1
    sar     bx, 1
    sar     word ds:12D6h, 1
    mov     cl, 4
    rol     bl, cl
    rol     byte ds:12DCh, cl
    ror     bl, cl
    ror     byte ds:12DCh, cl
    rcl     bl, cl
    rcl     byte ds:12DCh, cl
    rcr     bl, cl
    rcr     byte ds:12DCh, cl
    shl     bl, cl
    shl     byte ds:12DCh, cl
    shr     bl, cl
    shr     byte ds:12DCh, cl
    shl     bl, cl
    shl     byte ds:12DCh, cl
    sar     bl, cl
    sar     byte ds:12DCh, cl
    rol     bx, cl
    rol     word ds:12D6h, cl
    ror     bx, cl
    ror     word ds:12D6h, cl
    rcl     bx, cl
    rcl     word ds:12D6h, cl
    rcr     bx, cl
    rcr     word ds:12D6h, cl
    shl     bx, cl
    shl     word ds:12D6h, cl
    shr     bx, cl
    shr     word ds:12D6h, cl
    shl     bx, cl
    shl     word ds:12D6h, cl
    sar     bx, cl
    sar     word ds:12D6h, cl
    aad
    nop
    nop
    nop
    nop
    nop
    nop
    aam
    nop
    nop
    nop
    nop
    nop
    nop
    xlat
    mov     ax, 1234h
    mov     dx, 5678h
    cmc
    not     dl
    not     ax
    neg     dl
    neg     ax
    mov     dx, 20BDh
    mul     dx
    mov     bx, 2710h
    div     bx
    nop
    nop
    nop
    nop
    nop
    nop
    imul    dx
    nop
    nop
    nop
    nop
    nop
    nop
    idiv    bx
    clc
    stc
    pushf
    cld
    std
    popf
    mov     ax, 1234h
    mov     dx, 1234h
    mov     al, 12h
    mov     dl, 12h
    mov     ds:12D6h, ax
    mov     ds:12D6h, dx
    mov     ax, ds:12D6h
    mov     dx, ds:12D6h
    mov     ax, dx
    mov     dx, ax
    mov     al, dl
    mov     dl, al
    mov     dx, cs:[bx]
    ;mov     dx, [bp+var_s0]
    mov     dx, [bp+0]
    mov     dx, es:[si]
    mov     dx, [di]
    lea     bx, ds:0Ah
    push    word [bx]
    pop     word [bx]
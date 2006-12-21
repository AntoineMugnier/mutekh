# 1 "<built-in>"
# 1 "<command line>"
 .file "libgcc.s"


 .mode mshort




 .macro declare_near name
 .globl \name
 .type \name,@function
 .size \name,.Lend-\name
\name:
 .endm
 .macro ret
 rts
 .endm

 .macro farsym name
 .endm

 .macro declare name
 .globl \name
 .type \name,@function
 .size \name,.Lend-\name
\name:
 .endm

 .sect .softregs
 .globl _.tmp
 .globl _.z,_.xy
_.tmp: .dc.w 1; .type _.tmp,@object ; .size _.tmp,2
_.z: .dc.w 1; .type _.z,@object ; .size _.z,2
_.xy: .dc.w 1; .type _.xy,@object ; .size _.xy,2




 .sect .softregs
 .globl _.frame
_.frame: .dc.w 1; .type _.frame,@object ; .size _.frame,2

 .sect .text

 declare ___adddi3

 tsx
 xgdy
 ldd 8,x ; Add LSB
 addd 16,x
 std 6,y ; Save (carry preserved)

 ldd 6,x
 adcb 15,x
 adca 14,x
 std 4,y

 ldd 4,x
 adcb 13,x
 adca 12,x
 std 2,y

 ldd 2,x
 adcb 11,x ; Add MSB
 adca 10,x
 std 0,y

 xgdy
 ret



 declare ___subdi3

 tsx
 xgdy
 ldd 8,x ; Subtract LSB
 subd 16,x
 std 6,y ; Save, borrow preserved

 ldd 6,x
 sbcb 15,x
 sbca 14,x
 std 4,y

 ldd 4,x
 sbcb 13,x
 sbca 12,x
 std 2,y

 ldd 2,x ; Subtract MSB
 sbcb 11,x
 sbca 10,x
 std 0,y

 xgdy ;
 ret



 declare ___notdi2

 tsy
 xgdx
 ldd 8,y
 coma
 comb
 std 6,x

 ldd 6,y
 coma
 comb
 std 4,x

 ldd 4,y
 coma
 comb
 std 2,x

 ldd 2,y
 coma
 comb
 std 0,x
 xgdx
 ret



 declare_near ___negsi2

 comb
 coma
 xgdx
 comb
 coma
 inx
 xgdx
 bne done
 inx
done:
 rts



 declare_near ___one_cmplsi2

 comb
 coma
 xgdx
 comb
 coma
 xgdx
 rts



 declare_near ___ashlsi3

 xgdy
 clra
 andb #0x1f
 xgdy
 beq 2f
1:
 lsld
 xgdx
 rolb
 rola
 xgdx
 dey
 bne 1b
2:
 rts



 declare_near ___ashrsi3

 xgdy
 clra
 andb #0x1f
 xgdy
 beq 2f
1:
 xgdx
 asra
 rorb
 xgdx
 rora
 rorb
 dey
 bne 1b
2:
 rts



 declare_near ___lshrsi3

 xgdy
 clra
 andb #0x1f
 xgdy
 beq 2f
1:
 xgdx
 lsrd
 xgdx
 rora
 rorb
 dey
 bne 1b
2:
 rts



 declare_near ___lshrhi3

 cpx #16
 bge 3f
 cpx #0
 beq 2f
1:
 lsrd
 dex
 bne 1b
2:
 rts
3:
 clra
 clrb
 rts



 declare_near ___lshlhi3

 cpx #16
 bge 3f
 cpx #0
 beq 2f
1:
 lsld
 dex
 bne 1b
2:
 rts
3:
 clra
 clrb
 rts



 declare_near ___rotrhi3

___rotrhi3:
 xgdx
 clra
 andb #0x0f
 xgdx
 beq 2f
1:
 tap
 rorb
 rora
 dex
 bne 1b
2:
 rts



 declare_near ___rotlhi3

___rotlhi3:
 xgdx
 clra
 andb #0x0f
 xgdx
 beq 2f
1:
 asrb
 rolb
 rola
 rolb
 dex
 bne 1b
2:
 rts



 declare_near ___ashrhi3

 cpx #16
 bge Return_minus_1_or_zero
 cpx #0
 beq 2f
1:
 asra
 rorb
 dex
 bne 1b
2:
 rts
Return_minus_1_or_zero:
 clrb
 tsta
 bpl 3f
 comb
3:
 tba
 rts



 declare_near ___ashrqi3

 cmpa #8
 bge 4f
 tsta
 beq 2f
1:
 asrb
 deca
 bne 1b
2:
 rts
4:
 clrb
 tstb
 bpl 3f
 coma
3:
 tab
 rts



 declare_near ___lshlqi3

 cmpa #8
 bge 3f
 tsta
 beq 2f
1:
 lslb
 deca
 bne 1b
2:
 rts
3:
 clrb
 rts
 declare_near ___mulqi3

;
; short __mulqi3(signed char a, signed char b);
;
; signed char a -> register A
; signed char b -> register B
;
; returns the signed result of A * B in register D.
;
 tsta
 bmi A_neg
 tstb
 bmi B_neg
 mul
 rts
B_neg:
 negb
 bra A_or_B_neg
A_neg:
 nega
 tstb
 bmi AB_neg
A_or_B_neg:
 mul
 coma
 comb
 addd #1
 rts
AB_neg:
 negb
 mul
 rts



 declare_near ___mulhi3

;
;
; unsigned short ___mulhi3(unsigned short a, unsigned short b)
;
; a = register D
; b = register X
;

 pshx ; Preserve X
 exg x,y
 emul
 exg x,y
 pulx
 rts
;
;
; unsigned long __mulhi32(unsigned short a, unsigned short b)
;
; a = register D
; b = value on stack
;
; +---------------+
; | B low | <- 7,x
; +---------------+
; | B high | <- 6,x
; +---------------+
; | PC low |
; +---------------+
; | PC high |
; +---------------+
; | Tmp low |
; +---------------+
; | Tmp high |
; +---------------+
; | A low |
; +---------------+
; | A high |
; +---------------+ <- 0,x
;
;
; <B-low> 5,x
; <B-high> 4,x
; <ret> 2,x
; <A-low> 1,x
; <A-high> 0,x
;
 declare_near __mulhi32


 ldy 2,sp
 emul
 exg x,y
 rts
;
; <B-low> 8,y
; <B-high> 6,y
; <ret> 4,y
; <tmp> 2,y
; <A-low> 0,y
;
; D,X -> A
; Stack -> B
;
; The result is:
;
; (((A.low * B.high) + (A.high * B.low)) << 16) + (A.low * B.low)
;
;
;

 declare __mulsi3


 pshd ; Save A.low
 ldy 4,sp
 emul ; A.low * B.high
 ldy 6,sp
 exg x,d
 emul ; A.high * B.low
 leax d,x
 ldy 6,sp
 puld
 emul ; A.low * B.low
 exg d,y
 leax d,x
 exg d,y
 ret
;-----------------------------------------
; end required gcclib code
;-----------------------------------------

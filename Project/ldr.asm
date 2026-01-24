;
; ldr.asm - LDR value to Seven Segment Display
;
; Created: 29/12/2025
; Author : GitHub Copilot
; Device : ATmega32
; Crystal: 8MHz
;
; Connections:
; LDR: PA0 (ADC0)
; Seven Segment (Common Cathode): PORTC
;   PC0 -> a
;   PC1 -> b
;   PC2 -> c
;   PC3 -> d
;   PC4 -> e
;   PC5 -> f
;   PC6 -> g
;   PC7 -> dp (not used)

.include "m32def.inc"

.def temp = R16
.def adc_val = R17
.def display_val = R18

.cseg
.org 0x000
    rjmp RESET

RESET:
    ; Stack Pointer Setup
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp

    ; Initialize PORTC as output for Seven Segment
    ldi temp, 0xFF
    out DDRC, temp
    ldi temp, 0x00
    out PORTC, temp

    ; Initialize ADC
    ; ADMUX: AVCC reference, Left Adjust Result (8-bit in ADCH), Channel 0
    ldi temp, (1<<REFS0) | (1<<ADLAR)
    out ADMUX, temp

    ; ADCSRA: Enable ADC, Prescaler = 64 (8MHz/64 = 125kHz)
    ldi temp, (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1)
    out ADCSRA, temp

MAIN_LOOP:
    ; Start ADC conversion
    sbi ADCSRA, ADSC

WAIT_ADC:
    sbic ADCSRA, ADSC
    rjmp WAIT_ADC

    ; Read 8-bit result
    in adc_val, ADCH

    ; Scale 0-255 to 0-9
    ; display_val = adc_val / 26 (approx 255/10)
    mov temp, adc_val
    ldi display_val, 0
SCALE_LOOP:
    subi temp, 26
    brlo SCALE_DONE
    inc display_val
    cpi display_val, 9
    brne SCALE_LOOP
SCALE_DONE:

    ; Get Seven Segment Pattern
    ldi ZH, high(SEG_TABLE << 1)
    ldi ZL, low(SEG_TABLE << 1)
    add ZL, display_val
    clr temp
    adc ZH, temp
    lpm temp, Z

    ; Output to Seven Segment
    out PORTC, temp

    rjmp MAIN_LOOP

; Seven Segment Table (Common Cathode)
; 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
SEG_TABLE:
    .db 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F

;
; pwm_volt.asm - ADC Value to PWM Motor Control
;
; Created: 29/12/2025
; Device : ATmega32
; Crystal: 8MHz
;
; Description:
;   Reads analog voltage from PA6 (ADC6) using free-running mode
;   and controls motor speed via PWM based on ADC value
;
; Connections:
;   ADC Input:  PA6 (Potentiometer or voltage source 0-5V)
;   Motor PWM:  PD7 (OC2) -> L298N ENB
;   Motor Dir:  PB2 -> L298N IN3
;               PB4 -> L298N IN4
;
;-------------------------------------------------------------------------

.include "m32def.inc"

;-------------------------------------------------------------------------
; REGISTER DEFINITIONS
;-------------------------------------------------------------------------
.def tmp1   = R16       ; Temporary register
.def val    = R17       ; ADC value (motor speed)

;-------------------------------------------------------------------------
; PIN DEFINITIONS (L298N Motor B)
;-------------------------------------------------------------------------
.equ L298N_ENB  = 7     ; PD7 - ENB (PWM)
.equ L298N_IN3  = 2     ; PB2 - IN3
.equ L298N_IN4  = 4     ; PB4 - IN4

;-------------------------------------------------------------------------
; CODE SEGMENT
;-------------------------------------------------------------------------
.cseg

.org 0x000
    rjmp RESET

;-------------------------------------------------------------------------
; RESET
;-------------------------------------------------------------------------
.org 0x02A

RESET:
    ; Stack Pointer Setup
    ldi tmp1, low(RAMEND)
    out SPL, tmp1
    ldi tmp1, high(RAMEND)
    out SPH, tmp1

    ; Initialize Ports
    rcall INIT_PORTS
    
    ; Initialize ADC (Free-running mode)
    rcall INIT_ADC
    
    ; Initialize PWM
    rcall INIT_PWM
    
    ; Set motor direction: Forward
    sbi PORTB, L298N_IN3
    cbi PORTB, L298N_IN4

;-------------------------------------------------------------------------
; MAIN LOOP
;-------------------------------------------------------------------------
Loop:
    ; Read latest ADC value (8-bit, left-adjusted)
    in val, ADCH
    
    ; Output ADC value directly to PWM (motor speed)
    out OCR2, val
    
    ; Continue loop
    rjmp Loop

;-------------------------------------------------------------------------
; SUBROUTINE: INIT_PORTS
;-------------------------------------------------------------------------
INIT_PORTS:
    push tmp1

    ; PORTA - ADC Input (PA6)
    ldi tmp1, 0x00
    out DDRA, tmp1          ; All inputs
    out PORTA, tmp1         ; No pull-ups

    ; PORTB - Motor direction pins
    ; PB2 = IN3, PB4 = IN4
    ldi tmp1, (1<<L298N_IN3) | (1<<L298N_IN4)
    out DDRB, tmp1
    ldi tmp1, 0x00
    out PORTB, tmp1

    ; PORTD - PWM output
    ; PD7 = OC2 (ENB)
    ldi tmp1, (1<<L298N_ENB)
    out DDRD, tmp1

    pop tmp1
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: INIT_ADC
; Free-running mode, PA6, Left Adjust, AVCC reference
;-------------------------------------------------------------------------
INIT_ADC:
    push tmp1

    ; ADMUX Configuration:
    ; REFS0=1 (AVCC Reference)
    ; ADLAR=1 (Left Adjust - 8-bit result in ADCH)
    ; MUX=00110 (PA6)
    ldi tmp1, (1<<REFS0) | (1<<ADLAR) | (1<<MUX2) | (1<<MUX1)
    out ADMUX, tmp1
    
    ; ADCSRA Configuration:
    ; ADEN=1  (Enable ADC)
    ; ADATE=1 (Auto Trigger Enable - Free Running)
    ; ADSC=1  (Start Conversion)
    ; ADPS0=1 (Prescaler /2 -> 4MHz ADC clock)
    ; Note: For better accuracy use higher prescaler (ADPS2|ADPS1 = /64)
    ldi tmp1, (1<<ADEN) | (1<<ADATE) | (1<<ADSC) | (1<<ADPS0)
    out ADCSRA, tmp1

    pop tmp1
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: INIT_PWM
; Timer2 Fast PWM mode for motor speed control
;-------------------------------------------------------------------------
INIT_PWM:
    push tmp1

    ; TCCR2 Configuration:
    ; WGM21:WGM20 = 11 (Fast PWM)
    ; COM21:COM20 = 10 (Clear OC2 on Compare Match)
    ; CS22 = 1 (Prescaler = 64)
    ; PWM Frequency = 8MHz / 64 / 256 = 488 Hz
    ldi tmp1, (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS22)
    out TCCR2, tmp1

    ; Start with 0 duty cycle
    ldi tmp1, 0
    out OCR2, tmp1

    pop tmp1
    ret

;-------------------------------------------------------------------------
; END OF PROGRAM
;-------------------------------------------------------------------------

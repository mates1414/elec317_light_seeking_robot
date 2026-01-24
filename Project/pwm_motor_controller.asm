;
; pwm_motor_controller.asm - PWM Motor Controller for ATmega32 (EasyAVR7)
;
; Created: 29/12/2025
; Device : ATmega32
; Crystal: 8MHz
;
; Description:
;   Controls a DC motor speed using PWM via Timer0
;   Uses buttons to increase/decrease speed
;   Displays current speed level on LEDs
;
; Connections (EasyAVR7 with L298N Motor Driver):
;   L298N Motor B Connections:
;     ENB (PWM): PD7 (OC2) -> L298N ENB pin (Motor B speed control)
;     IN3:       PB2       -> L298N IN3 pin (Motor B direction)
;     IN4:       PB4       -> L298N IN4 pin (Motor B direction)
;
;   Direction Logic (Motor B):
;     IN3=HIGH, IN4=LOW  -> Forward
;     IN3=LOW,  IN4=HIGH -> Reverse
;     IN3=IN4            -> Stop (Brake)
;
;   Buttons:
;     Speed Up Button:   PD0 (Active Low with Pull-up)
;     Speed Down Button: PD1 (Active Low with Pull-up)
;     Direction Button:  PD2 (Active Low with Pull-up)
;     Stop Button:       PD3 (Active Low with Pull-up)
;
;   Speed LEDs: PORTC (8 LEDs showing speed level)
;
;-------------------------------------------------------------------------

.include "m32def.inc"

;-------------------------------------------------------------------------
; REGISTER DEFINITIONS
;-------------------------------------------------------------------------
.def temp       = R16       ; General purpose temporary register
.def speed      = R17       ; Current motor speed (0-255)
.def direction  = R18       ; Motor direction (0 = Forward, 1 = Reverse)
.def btn_state  = R19       ; Button state register
.def debounce   = R20       ; Debounce counter

;-------------------------------------------------------------------------
; CONSTANTS
;-------------------------------------------------------------------------
.equ SPEED_STEP = 32        ; Speed increment/decrement step (8 levels)
.equ SPEED_MIN  = 0         ; Minimum speed (motor off)
.equ SPEED_MAX  = 255       ; Maximum speed

.equ BTN_UP     = 0         ; PD0 - Speed Up button
.equ BTN_DOWN   = 1         ; PD1 - Speed Down button
.equ BTN_DIR    = 2         ; PD2 - Direction toggle button
.equ BTN_STOP   = 3         ; PD3 - Emergency stop button

; L298N Motor Driver Pins (Motor B)
.equ L298N_ENB  = 7         ; PD7 - ENB (PWM speed control via OC2)
.equ L298N_IN3  = 2         ; PB2 - IN3 (Direction control)
.equ L298N_IN4  = 4         ; PB4 - IN4 (Direction control)

;-------------------------------------------------------------------------
; CODE SEGMENT
;-------------------------------------------------------------------------
.cseg

;-------------------------------------------------------------------------
; INTERRUPT VECTOR TABLE
;-------------------------------------------------------------------------
.org 0x000
    rjmp RESET

;-------------------------------------------------------------------------
; RESET - Main Program Entry Point
;-------------------------------------------------------------------------
.org 0x02A

RESET:
    ; STACK POINTER SETUP
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp

    ; Initialize registers
    ldi speed, 128          ; Start with 50% duty cycle
    ldi direction, 0        ; Forward direction

    ; Initialize peripherals (ORDER IS IMPORTANT!)
    rcall INIT_PORTS        ; 1. Setup I/O pins
    rcall SET_MOTOR_FORWARD ; 2. Set direction BEFORE PWM
    rcall INIT_PWM          ; 3. Start PWM after direction is set
    rcall DELAY             ; 4. Small delay for motor driver to stabilize

;-------------------------------------------------------------------------
; MAIN LOOP
;-------------------------------------------------------------------------
MAIN_LOOP:
    ; Ensure PWM is always running at set speed
    out OCR2, speed
    
    ; rcall READ_BUTTONS      ; DISABLED - Buttons not used for testing
    rcall UPDATE_LEDS       ; Update speed display
    rcall SHORT_DELAY       ; Short debounce delay
    rjmp MAIN_LOOP

;-------------------------------------------------------------------------
; SUBROUTINE: INIT_PORTS
; Purpose: Configure I/O pins
;-------------------------------------------------------------------------
INIT_PORTS:
    push temp

    ; PORTB Configuration for L298N Motor B
    ; PB2 = IN3 (Direction control)
    ; PB4 = IN4 (Direction control)
    ldi temp, (1<<L298N_IN3) | (1<<L298N_IN4)
    out DDRB, temp
    
    ; Start with motor stopped (IN3=LOW, IN4=LOW = Coast/Free)
    ldi temp, 0x00
    out PORTB, temp

    ; PORTC Configuration - All outputs for LED display
    ldi temp, 0xFF
    out DDRC, temp
    ldi temp, 0x00
    out PORTC, temp

    ; PORTD Configuration
    ; PD0-PD3: Inputs with pull-ups for buttons
    ; PD7: Output for OC2 (ENB - Motor B PWM)
    ldi temp, (1<<L298N_ENB)
    out DDRD, temp          ; PD7 as output, rest as inputs
    ldi temp, (1<<BTN_UP) | (1<<BTN_DOWN) | (1<<BTN_DIR) | (1<<BTN_STOP)
    out PORTD, temp         ; Enable pull-ups on button pins

    pop temp
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: SET_MOTOR_FORWARD
; Purpose: Set motor direction to forward
;-------------------------------------------------------------------------
SET_MOTOR_FORWARD:
    ; Forward: IN3=HIGH, IN4=LOW
    sbi PORTB, L298N_IN3
    cbi PORTB, L298N_IN4
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: SET_MOTOR_REVERSE  
; Purpose: Set motor direction to reverse
;-------------------------------------------------------------------------
SET_MOTOR_REVERSE:
    ; Reverse: IN3=LOW, IN4=HIGH
    cbi PORTB, L298N_IN3
    sbi PORTB, L298N_IN4
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: SET_MOTOR_BRAKE
; Purpose: Brake motor (fast stop)
;-------------------------------------------------------------------------
SET_MOTOR_BRAKE:
    ; Brake: IN3=HIGH, IN4=HIGH (according to L298N datasheet)
    sbi PORTB, L298N_IN3
    sbi PORTB, L298N_IN4
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: INIT_PWM
; Purpose: Configure Timer0 for Fast PWM mode
;-------------------------------------------------------------------------
INIT_PWM:
    push temp

    ; TCCR2 Configuration (Timer2 for OC2/PD7 - ENB):
    ; WGM21:WGM20 = 11 (Fast PWM mode)
    ; COM21:COM20 = 10 (Clear OC2 on Compare Match, Set at BOTTOM)
    ; CS22:CS21:CS20 = 100 (Prescaler = 64)
    ; PWM Frequency = 8MHz / 64 / 256 = 488 Hz
    ldi temp, (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS22)
    out TCCR2, temp

    ; Initialize OCR2 with current speed (0)
    out OCR2, speed

    pop temp
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: READ_BUTTONS
; Purpose: Check button states and adjust speed/direction
; Uses double-read for noise immunity
;-------------------------------------------------------------------------
READ_BUTTONS:
    push temp
    push R21

    ; First read
    in btn_state, PIND
    
    ; Small delay for debounce
    ldi R21, 50
BTN_DEBOUNCE:
    dec R21
    brne BTN_DEBOUNCE
    
    ; Second read - must match first read
    in temp, PIND
    cp btn_state, temp
    brne READ_BUTTONS_DONE  ; If readings differ, ignore (noise)

    ; Check Speed Up button (PD0) - Active Low
    sbrs btn_state, BTN_UP
    rcall SPEED_UP

    ; Check Speed Down button (PD1) - Active Low
    sbrs btn_state, BTN_DOWN
    rcall SPEED_DOWN

    ; Check Direction button (PD2) - Active Low
    sbrs btn_state, BTN_DIR
    rcall TOGGLE_DIRECTION

    ; Check Stop button (PD3) - Active Low (Emergency Stop)
    sbrs btn_state, BTN_STOP
    rcall MOTOR_STOP

READ_BUTTONS_DONE:
    ; Update PWM duty cycle (ENB - Motor B)
    out OCR2, speed

    pop R21
    pop temp
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: SPEED_UP
; Purpose: Increase motor speed
;-------------------------------------------------------------------------
SPEED_UP:
    push temp

    ; Check if already at maximum
    cpi speed, SPEED_MAX
    breq SPEED_UP_DONE

    ; Add speed step
    ldi temp, SPEED_STEP
    add speed, temp

    ; Check for overflow (wrap around)
    brcc SPEED_UP_DONE
    ldi speed, SPEED_MAX    ; Cap at maximum

SPEED_UP_DONE:
    pop temp
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: SPEED_DOWN
; Purpose: Decrease motor speed
;-------------------------------------------------------------------------
SPEED_DOWN:
    push temp

    ; Check if already at minimum
    cpi speed, SPEED_MIN
    breq SPEED_DOWN_DONE

    ; Subtract speed step
    ldi temp, SPEED_STEP
    sub speed, temp

    ; Check for underflow (wrap around)
    brcc SPEED_DOWN_DONE
    ldi speed, SPEED_MIN    ; Cap at minimum

SPEED_DOWN_DONE:
    pop temp
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: TOGGLE_DIRECTION
; Purpose: Toggle motor direction using L298N Motor B (IN3, IN4)
; Forward: IN3=HIGH, IN4=LOW
; Reverse: IN3=LOW,  IN4=HIGH
;-------------------------------------------------------------------------
TOGGLE_DIRECTION:
    push temp

    ; Toggle direction register
    ldi temp, 0x01
    eor direction, temp

    ; Update L298N direction pins (Motor B)
    sbrc direction, 0
    rcall SET_MOTOR_REVERSE
    sbrs direction, 0
    rcall SET_MOTOR_FORWARD

    ; Wait for button release (simple debounce)
    rcall LONG_DELAY

    pop temp
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: MOTOR_STOP
; Purpose: Emergency stop - Brake motor using L298N Motor B
; Brake: IN3=HIGH, IN4=HIGH (L298N datasheet: fast motor stop)
;-------------------------------------------------------------------------
MOTOR_STOP:
    push temp

    ; Set speed to 0
    ldi speed, 0
    out OCR2, speed

    ; Brake: IN3=HIGH, IN4=HIGH (fast stop per L298N datasheet)
    rcall SET_MOTOR_BRAKE

    ; Wait for button release
    rcall LONG_DELAY

    ; Restore direction after stop
    sbrc direction, 0
    rcall SET_MOTOR_REVERSE
    sbrs direction, 0
    rcall SET_MOTOR_FORWARD

    pop temp
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: UPDATE_LEDS
; Purpose: Display current speed level on LEDs
;-------------------------------------------------------------------------
UPDATE_LEDS:
    push temp

    ; Map speed (0-255) to LED bar display
    ; 8 LEDs representing 8 speed levels
    mov temp, speed

    ; Create LED pattern based on speed
    cpi temp, 32
    brlo LED_0
    cpi temp, 64
    brlo LED_1
    cpi temp, 96
    brlo LED_2
    cpi temp, 128
    brlo LED_3
    cpi temp, 160
    brlo LED_4
    cpi temp, 192
    brlo LED_5
    cpi temp, 224
    brlo LED_6
    rjmp LED_7

LED_0:
    ldi temp, 0b00000000    ; No LEDs
    rjmp LED_UPDATE
LED_1:
    ldi temp, 0b00000001    ; 1 LED
    rjmp LED_UPDATE
LED_2:
    ldi temp, 0b00000011    ; 2 LEDs
    rjmp LED_UPDATE
LED_3:
    ldi temp, 0b00000111    ; 3 LEDs
    rjmp LED_UPDATE
LED_4:
    ldi temp, 0b00001111    ; 4 LEDs
    rjmp LED_UPDATE
LED_5:
    ldi temp, 0b00011111    ; 5 LEDs
    rjmp LED_UPDATE
LED_6:
    ldi temp, 0b00111111    ; 6 LEDs
    rjmp LED_UPDATE
LED_7:
    ldi temp, 0b01111111    ; 7 LEDs
    ; Check for full speed
    cpi speed, 255
    brne LED_UPDATE
    ldi temp, 0b11111111    ; All 8 LEDs

LED_UPDATE:
    out PORTC, temp

    pop temp
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: SHORT_DELAY
; Purpose: Very short delay (~5ms) - doesn't block motor
;-------------------------------------------------------------------------
SHORT_DELAY:
    push temp
    push R21

    ldi temp, 25
SHORT_OUTER:
    ldi R21, 255
SHORT_INNER:
    dec R21
    brne SHORT_INNER
    dec temp
    brne SHORT_OUTER

    pop R21
    pop temp
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: DELAY
; Purpose: Short delay for button debounce (~20ms)
;-------------------------------------------------------------------------
DELAY:
    push temp
    push R21
    push R22

    ldi temp, 100
DELAY_OUTER:
    ldi R21, 255
DELAY_MIDDLE:
    ldi R22, 3
DELAY_INNER:
    dec R22
    brne DELAY_INNER
    dec R21
    brne DELAY_MIDDLE
    dec temp
    brne DELAY_OUTER

    pop R22
    pop R21
    pop temp
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: LONG_DELAY
; Purpose: Longer delay for direction button (~200ms)
;-------------------------------------------------------------------------
LONG_DELAY:
    push temp

    ldi temp, 10
LONG_DELAY_LOOP:
    rcall DELAY
    dec temp
    brne LONG_DELAY_LOOP

    pop temp
    ret

;-------------------------------------------------------------------------
; END OF PROGRAM
;-------------------------------------------------------------------------

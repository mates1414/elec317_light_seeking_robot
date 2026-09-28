;-------------------------------------------------------------------------
; File Name: main_tank_turn_direct_control.asm
; Created: 17/12/2025 20:38:29
; Author : ates1
; Device : ATmega328P (Arduino Uno)
; Crystal: 16MHz
;-------------------------------------------------------------------------
; Ported from ATmega32 (EasyAVR7, L298N, multiplexed 7-seg) to
; Arduino Uno (ATmega328P, TB6612FNG, TM1637 4-digit display)
;-------------------------------------------------------------------------
.include "m328Pdef.inc"
;-------------------------------------------------------------------------
; REGISTER DEFINITIONS
;-------------------------------------------------------------------------
.def temp = R16         ; Temporary register
.def temp2 = R17        ; Temporary register 2
.def sensor_left = R18  ; Left sensor value
.def sensor_right = R19 ; Right sensor value
.def motor_left = R20   ; Left motor speed (PWM value)
.def motor_right = R21  ; Right motor speed (PWM value)
.def robot_state = R22	; $00 close / $FF open
.def dark_mode = R23	; $00 dark  / $FF bright
.def disp0 = R24        ; Seven segment display digit 0
.def disp1 = R25        ; Seven segment display digit 1. Digit 0 and 1 are for right sensor
.def disp2 = R26        ; Seven segment display digit 2
.def disp3 = R27        ; Seven segment display digit 3. Digit 2 and 3 are for left sensor
.def threshold = R28    ; Light threshold for decision making. Can be changeable via button/dark mode
.def bit_count = R29    ; TM1637 bit counter
;-------------------------------------------------------------------------
; CONSTANTS
;-------------------------------------------------------------------------
.equ motor_turn  = 96       ; PWM value for turning
.equ motor_on  = 192        ; PWM value for moving forward (~6.3V from a full 2S 18650 pack)
.equ motor_off = 0          ; PWM value for stopping
.equ compare_threshold = 24 ; Threshold difference between sensors to trigger turn
.equ dark_threshold = 32    ; Light threshold for dark mode
.equ bright_threshold = 160 ; Light threshold for bright mode
.equ B_controller = 16      ; Left motor speed compensation.
; Two motors do not run at the same speed with the same PWM, recalibrate on the new chassis.
.equ btn_confirm = 125      ; Button must stay LOW for 125 x 80us = 10ms to count as a press
.equ btn_lockout = 250      ; Ignore a button for 250 Timer0 overflows (~256ms) after a press
;-------------------------------------------------------------------------
; SRAM VARIABLES
;-------------------------------------------------------------------------
.dseg
.org SRAM_START
lock_start: .byte 1         ; Start/Stop button lockout counter (Timer0 overflows)
lock_mode:  .byte 1         ; Mode button lockout counter (Timer0 overflows)
;-------------------------------------------------------------------------
; TB6612FNG MOTOR DRIVER PIN DEFINITIONS (STBY -> 5V)
;-------------------------------------------------------------------------
; Motor A (Right Motor)
.equ TB6612_PWMA = 6        ; PD6 / D6  - PWMA (PWM via OC0A) - Right Motor
.equ TB6612_AIN1 = 7        ; PD7 / D7  - AIN1 (Right Motor Direction)
.equ TB6612_AIN2 = 0        ; PB0 / D8  - AIN2 (Right Motor Direction)

; Motor B (Left Motor)
.equ TB6612_PWMB = 3        ; PB3 / D11 - PWMB (PWM via OC2A) - Left Motor
.equ TB6612_BIN1 = 4        ; PD4 / D4  - BIN1 (Left Motor Direction)
.equ TB6612_BIN2 = 4        ; PB4 / D12 - BIN2 (Left Motor Direction)
;-------------------------------------------------------------------------
; OTHER PIN DEFINITIONS
;-------------------------------------------------------------------------
.equ BTN_START  = 2         ; PD2 / D2  - INT0 Start/Stop button (to GND, internal pull-up)
.equ BTN_MODE   = 3         ; PD3 / D3  - INT1 Dark/Bright mode button (to GND, internal pull-up)
.equ LDR_RIGHT  = 0         ; PC0 / A0  - ADC0 Right LDR
.equ LDR_LEFT   = 1         ; PC1 / A1  - ADC1 Left LDR
.equ LED_RUN    = 2         ; PC2 / A2  - Running status LED
.equ LED_BRIGHT = 3         ; PC3 / A3  - Bright mode status LED
.equ TM1637_CLK = 1         ; PB1 / D9  - TM1637 CLK
.equ TM1637_DIO = 2         ; PB2 / D10 - TM1637 DIO
;-------------------------------------------------------------------------
; TM1637 COMMANDS
;-------------------------------------------------------------------------
.equ TM1637_CMD_DATA    = $40   ; Write data, auto address increment
.equ TM1637_CMD_ADDR    = $C0   ; Start address = leftmost digit
.equ TM1637_CMD_DISPLAY = $8F   ; Display on, brightness 7 (max)

.cseg
.org 0x0000
    rjmp RESET

.org INT0addr
    jmp INT0_ISR          ; INT0 - Start/Stop Button (PD2)

.org INT1addr
    jmp INT1_ISR          ; INT1 - Dark/Bright Mode Button (PD3)

.org OVF2addr
    jmp TIM2_OVF

.org OVF0addr
    jmp TIM0_OVF

.org INT_VECTORS_SIZE
;-------------------------------------------------------------------------
; RESET
;-------------------------------------------------------------------------
RESET:
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp

    ; INIT PORTS
	rcall INIT_PORTS

	; INIT ADC
	rcall INIT_ADC

    ; INIT PWM
	rcall INIT_PWM

	; INIT External Interrupts for buttons
	rcall INIT_INTERRUPTS

	; INIT display values
    ; initialize display values
    ; rcall INIT_DISP

	ldi threshold, bright_threshold

	; initialize robot state (0xFF = running, 0x00 = stopped)
	ldi robot_state, $00
	; initialize dark mode (0xFF = bright mode, 0x00 = dark mode)
	ldi dark_mode, $FF
	; motors start from standstill, buttons start unlocked
	ldi motor_right, motor_off
	ldi motor_left, motor_off
	clr temp
	sts lock_start, temp
	sts lock_mode, temp

	sei
;-------------------------------------------------------------------------
; MAIN_LOOP
;-------------------------------------------------------------------------
MAIN_LOOP:
	; read button / PD2, PD3 interrupt button read

    ; mode indication leds
    rcall open_led_bright
    rcall open_led_run

    ; read right sensor
	rcall read_right_sensor

    ; read left sensor
	rcall read_left_sensor

    ; decision making
	rcall compare_sensors

    ; update display
    rcall sensor_to_display_values

	; display sensor data on seven segment
	rcall display_sensor_data

	; PWM registers follow motor_right/motor_left in TIM0_OVF (soft start)

    rjmp MAIN_LOOP
;-------------------------------------------------------------------------
open_led_bright:
    ; open led on PC3 (A3) if robot is in bright mode
    cpi dark_mode, $00
    breq close_led_bright
    sbi PORTC, LED_BRIGHT
    rjmp end_led_bright
close_led_bright:
    cbi PORTC, LED_BRIGHT
end_led_bright:
    ret
;-------------------------------------------------------------------------
open_led_run:
    ; open led on PC2 (A2) if robot is running
    cpi robot_state, $00
    breq close_led_run
    sbi PORTC, LED_RUN
    rjmp end_led_run
close_led_run:
    cbi PORTC, LED_RUN
end_led_run:
    ret
;-------------------------------------------------------------------------
; INIT SUBROUTINES
;-------------------------------------------------------------------------
INIT_PORTS:
	; PORTB - TB6612 Motor B + TM1637
    ; PB3 = OC2A (PWMB - Left Motor PWM)
    ; PB0 = AIN2 (Right Motor Direction)
    ; PB4 = BIN2 (Left Motor Direction)
    ; PB1/PB2 = TM1637 CLK/DIO, driven open-drain through DDRB (PORTB bit stays 0)
    ;           input = released (module pull-up -> HIGH), output = LOW
    ldi temp, (1<<TB6612_PWMB) | (1<<TB6612_AIN2) | (1<<TB6612_BIN2)
    out DDRB, temp          ; Set as outputs, TM1637 lines released
	ldi temp, $00
	out PORTB, temp         ; AIN2=LOW, BIN2=LOW (ileri)

    ; PORTC - ADC Input + Status LEDs
    ; PC0 = Right LDR sensor (ADC0)
    ; PC1 = Left LDR sensor (ADC1)
    ; PC2, PC3 = Status LEDs
    ldi temp, (1<<LED_RUN) | (1<<LED_BRIGHT)
    out DDRC, temp          ; PC2-PC3 outputs, PC0-PC1 inputs (ADC)
    ldi temp, $00
    out PORTC, temp         ; No pull-ups on ADC pins (external circuit), LEDs off initially

	; PORTD - Right Motor PWM + Motor Directions + Button Inputs
	; PD6 = OC0A (PWMA - Right Motor PWM)
	; PD7 = AIN1 (Right Motor Direction)
	; PD4 = BIN1 (Left Motor Direction)
	; PD2 = INT0 (Start/Stop Button)
	; PD3 = INT1 (Dark/Bright Mode Button)
	; PD0, PD1 = USART RX/TX, left free for USB upload
    ldi temp, (1<<TB6612_PWMA) | (1<<TB6612_AIN1) | (1<<TB6612_BIN1)
    out DDRD, temp          ; PD6, PD7, PD4 as outputs, PD2/PD3 as inputs

	; AIN1=HIGH, BIN1=HIGH (ileri), enable pull-ups for buttons
	ldi temp, (1<<TB6612_AIN1) | (1<<TB6612_BIN1) | (1<<BTN_START) | (1<<BTN_MODE)
	out PORTD, temp
	ret
;-------------------------------------------------------------------------
INIT_ADC:
    ; ADMUX: AVCC reference, Left Adjust Result (8-bit in ADCH)
    ldi temp, (1<<REFS0) | (1<<ADLAR)
    sts ADMUX, temp

    ; ADCSRA: Enable ADC, Prescaler = 128 (16MHz/128 = 125kHz ADC clock)
    ldi temp, (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)
    sts ADCSRA, temp

    ; Disable digital input buffers on LDR pins
    ldi temp, (1<<ADC0D) | (1<<ADC1D)
    sts DIDR0, temp

	ret
;-------------------------------------------------------------------------
INIT_PWM:
    ; TIMER0 SETUP - Right Motor PWM (OC0A = PD6 / D6)
    ; TCCR0A/TCCR0B: Fast PWM, Non-Inverting, Prescaler = 64
    ; 16MHz / 64 / 256 = ~976Hz PWM
	ldi temp, (1<<WGM00) | (1<<WGM01) | (1<<COM0A1)
    out TCCR0A, temp
	ldi temp, (1<<CS01) | (1<<CS00)
    out TCCR0B, temp
    ; Initialize duty cycle to 0
    ldi temp, 0
    out OCR0A, temp

    ; TIMER2 SETUP - Left Motor PWM (OC2A = PB3 / D11)
    ; TCCR2A/TCCR2B: Fast PWM, Non-Inverting, Prescaler = 64
    ldi temp, (1<<WGM20) | (1<<WGM21) | (1<<COM2A1)
    sts TCCR2A, temp
    ldi temp, (1<<CS22)
    sts TCCR2B, temp
    ; Initialize duty cycle to 0
    ldi temp, 0
    sts OCR2A, temp

    ; Timer0 overflow (~1ms tick) drives the PWM ramp and button lockouts
    ldi temp, (1<<TOIE0)
    sts TIMSK0, temp

    ret
;-------------------------------------------------------------------------
; INIT_INTERRUPTS
; Purpose: Configure INT0 (PD2) and INT1 (PD3) for button inputs
;          Falling edge triggered (active-low buttons with pull-ups)
;-------------------------------------------------------------------------
INIT_INTERRUPTS:
    ; EICRA: Set INT0 and INT1 to falling edge trigger
    ; ISC01=1, ISC00=0 -> INT0 falling edge
    ; ISC11=1, ISC10=0 -> INT1 falling edge
    ldi temp, (1<<ISC01) | (1<<ISC11)
    sts EICRA, temp

    ; Clear any pending interrupt flags (write 1 to clear)
    ldi temp, (1<<INTF0) | (1<<INTF1)
    out EIFR, temp

    ; EIMSK: Enable INT0 and INT1
    ldi temp, (1<<INT0) | (1<<INT1)
    out EIMSK, temp

    ret
;-------------------------------------------------------------------------
; INIT_DISP
;-------------------------------------------------------------------------
;INIT_DISP:
;    ldi disp0, 9
;    ldi disp1, 9
;    ldi disp2, 9
;    ldi disp3, 9
;    ret
;-------------------------------------------------------------------------
; SUBROUTINE: READ_RIGHT_SENSOR
; Purpose: Read ADC Channel 0 (PC0 / A0 - Right LDR) and store in sensor_right
; ADMUX/ADCSRA/ADCH are in extended I/O on ATmega328P -> lds/sts
;-------------------------------------------------------------------------
read_right_sensor:
    ; Select ADC Channel 0 (A0 - Right Sensor)
    ldi temp, (1<<REFS0) | (1<<ADLAR) | LDR_RIGHT
    sts ADMUX, temp

    ; Start conversion
    lds temp, ADCSRA
    ori temp, (1<<ADSC)
    sts ADCSRA, temp

read_right_wait:
    ; Wait for conversion to complete
    lds temp, ADCSRA
    sbrc temp, ADSC
    rjmp read_right_wait

    ; Read result (8-bit from ADCH due to Left Adjust)
    lds sensor_right, ADCH

    ret
;-------------------------------------------------------------------------
; SUBROUTINE: READ_LEFT_SENSOR
; Purpose: Read ADC Channel 1 (PC1 / A1 - Left LDR) and store in sensor_left
;-------------------------------------------------------------------------
read_left_sensor:
    ; Select ADC Channel 1 (A1 - Left Sensor)
    ldi temp, (1<<REFS0) | (1<<ADLAR) | LDR_LEFT
    sts ADMUX, temp

    ; Start conversion
    lds temp, ADCSRA
    ori temp, (1<<ADSC)
    sts ADCSRA, temp

read_left_wait:
    ; Wait for conversion to complete
    lds temp, ADCSRA
    sbrc temp, ADSC
    rjmp read_left_wait

    ; Read result (8-bit from ADCH due to Left Adjust)
    lds sensor_left, ADCH

    ret
;-------------------------------------------------------------------------
; SUBROUTINE: COMPARE_SENSORS
; Tank Turn: During turns, one motor goes forward, other goes backward
;-------------------------------------------------------------------------
compare_sensors:
    ; Compare sensor values and set motor speeds accordingly
	cpi robot_state, $00
    breq zero_zero ; If stopped, turn off motors
    cp sensor_right, threshold  ; Compare right sensor with threshold
	brlo right_0                ; if right < threshold go to right_0
right_1:                        ; else right >= threshold go to right_1
	cp sensor_left, threshold   ; Compare left sensor with threshold
	brsh one_one                ; if left >= threshold go to one_one
	mov temp, sensor_right      ; now we know right >= threshold and left < threshold
	sub temp, sensor_left
	cpi temp, compare_threshold ; compare sensor difference with compare_threshold
	brlo only_1_ldr_turn        ; if difference < compare_threshold go to only_1_ldr_turn which is proportional turn
one_zero:                       ; else difference >= compare_threshold go to one_zero
	; Right sees light, left dark - turn RIGHT (tank turn)
	; Left motor forward, right motor backward
	rcall set_tank_turn_right   ; tank turn right
	ret
one_one:
	; Both sensors see light - go forward
    mov temp, sensor_left       ; find which sensor is bigger
    sub temp, sensor_right
    cpi temp, $00
    brsh left_bigger
    com temp
right_bigger:                   ; right sensor bigger
    cpi temp, compare_threshold ; if difference < compare_threshold go to equal
    brlo equal                  ; set motors forward
    ; Right sensor significantly bigger - turn RIGHT
    rcall set_motors_turn       ; proportional turn
    rjmp one_one_end
left_bigger:
    cpi temp, compare_threshold ; left sensor bigger
    brlo equal                  ; if difference < compare_threshold go to equal
    rcall set_motors_turn       ; proportional turn
    rjmp one_one_end
equal:
    rcall set_motors_forward    ; go forward
one_one_end:
	ret
only_1_ldr_turn:
    rcall set_motors_turn
    ret
right_0:
	cp sensor_left, threshold   ; Compare left sensor with threshold
	brlo zero_zero              ; if left < threshold go to zero_zero
	mov temp, sensor_left
	sub temp, sensor_right
	cpi temp, compare_threshold ; compare sensor difference with compare_threshold
	brlo only_1_ldr_turn        ; if difference < compare_threshold go to only_1_ldr_turn which is proportional turn
zero_one:
	; Left sees light, right dark - turn LEFT (tank turn)
	; Right motor forward, left motor backward
	rcall set_tank_turn_left    ; tank turn left
	ret

zero_zero:
	; No light - stop
	rcall set_motors_stop       ; stop motors
    ret
;-------------------------------------------------------------------------
; Subroutine: store sensor data to modify and set them to motor PWM values
sensor_to_motor:
    mov temp, sensor_right
    mov temp2, sensor_left
    lsr temp
    lsr temp2
    subi temp, - $40
    subi temp2, - $40
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: SET_MOTORS_FORWARD
; Purpose: Set both motors to forward direction
; Right Motor: AIN1=HIGH, AIN2=LOW
; Left Motor:  BIN1=HIGH, BIN2=LOW
;-------------------------------------------------------------------------
set_motors_forward:
    rcall cut_pwm_if_right_backward
    rcall cut_pwm_if_left_backward
    sbi PORTD, TB6612_AIN1
    cbi PORTB, TB6612_AIN2
    sbi PORTD, TB6612_BIN1
    cbi PORTB, TB6612_BIN2

    ldi motor_right, motor_on
    ldi motor_left, motor_on + B_controller
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: SET_MOTORS_TURN
; Purpose: Set both motors to forward direction
; Right Motor: AIN1=HIGH, AIN2=LOW
; Left Motor:  BIN1=HIGH, BIN2=LOW
;-------------------------------------------------------------------------
set_motors_turn:
    rcall cut_pwm_if_right_backward
    rcall cut_pwm_if_left_backward
    sbi PORTD, TB6612_AIN1
    cbi PORTB, TB6612_AIN2
    sbi PORTD, TB6612_BIN1
    cbi PORTB, TB6612_BIN2
    ; proportional control
    rcall sensor_to_motor   ; adjust sensor values to motor pwm values so they can initiate the movement
    mov motor_right, temp2  ; right sensor value -> left motor speed
	mov motor_left, temp    ; right sensor value -> left motor speed
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: SET_TANK_TURN_LEFT
; Purpose: Tank turn left - Right forward, Left backward
; Right Motor: AIN1=HIGH, AIN2=LOW  (forward)
; Left Motor:  BIN1=LOW,  BIN2=HIGH (backward)
;-------------------------------------------------------------------------
set_tank_turn_left:
    rcall cut_pwm_if_right_backward
    rcall cut_pwm_if_left_forward
    sbi PORTD, TB6612_AIN1
    cbi PORTB, TB6612_AIN2
    cbi PORTD, TB6612_BIN1
    sbi PORTB, TB6612_BIN2

    ldi motor_right, motor_turn
	ldi motor_left, motor_turn + B_controller
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: SET_TANK_TURN_RIGHT
; Purpose: Tank turn right - Left forward, Right backward
; Right Motor: AIN1=LOW,  AIN2=HIGH (backward)
; Left Motor:  BIN1=HIGH, BIN2=LOW  (forward)
;-------------------------------------------------------------------------
set_tank_turn_right:
    rcall cut_pwm_if_right_forward
    rcall cut_pwm_if_left_backward
    cbi PORTD, TB6612_AIN1
    sbi PORTB, TB6612_AIN2
    sbi PORTD, TB6612_BIN1
    cbi PORTB, TB6612_BIN2

    ldi motor_right, motor_turn
	ldi motor_left, motor_turn + B_controller
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: SET_MOTORS_STOP
; Purpose: Stop both motors (coast mode)
; Both IN pins LOW = coast, Both HIGH = brake
;-------------------------------------------------------------------------
set_motors_stop:
    cbi PORTD, TB6612_AIN1
    cbi PORTB, TB6612_AIN2
    cbi PORTD, TB6612_BIN1
    cbi PORTB, TB6612_BIN2

	ldi motor_right, motor_off
	ldi motor_left, motor_off
    ret
;-------------------------------------------------------------------------
; SUBROUTINES: CUT_PWM_IF_*
; Purpose: Before a motor reverses, drop its PWM to 0 so TIM0_OVF ramps it
;          up again. Reversing a spinning motor at full PWM draws a current
;          spike large enough to disturb the rest of the circuit.
;-------------------------------------------------------------------------
cut_pwm_if_right_forward:
    clr temp
    sbic PORTD, TB6612_AIN1     ; AIN1 HIGH = right motor going forward
    out OCR0A, temp
    ret

cut_pwm_if_right_backward:
    clr temp
    sbic PORTB, TB6612_AIN2     ; AIN2 HIGH = right motor going backward
    out OCR0A, temp
    ret

cut_pwm_if_left_forward:
    clr temp
    sbic PORTD, TB6612_BIN1     ; BIN1 HIGH = left motor going forward
    sts OCR2A, temp
    ret

cut_pwm_if_left_backward:
    clr temp
    sbic PORTB, TB6612_BIN2     ; BIN2 HIGH = left motor going backward
    sts OCR2A, temp
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: SENSOR_TO_DISPLAY_VALUES
;-------------------------------------------------------------------------
sensor_to_display_values:
; Scale sensor values (0-255) to display values (0-63)
; right sensor
right:
    mov temp, sensor_right
    lsr temp
    lsr temp

    ldi disp1, $00
    cpi temp, $0A
    brlo right_done
right_ten:
    subi temp, $0A
    inc disp1
    cpi temp, $0A
    brsh right_ten
right_done:
    mov disp0, temp
; left sensor
left:
    mov temp, sensor_left
    lsr temp
    lsr temp
    ldi disp3, $00
    cpi temp, $0A
    brlo left_done
left_ten:
    subi temp, $0A
    inc disp3
    cpi temp, $0A
    brsh left_ten
left_done:
    mov disp2, temp

    ret
;-------------------------------------------------------------------------
; SUBROUTINE: DISPLAY_SENSOR_DATA
;-------------------------------------------------------------------------
display_sensor_data:
    ; For demonstration puposes, display sensor data on TM1637 seven segment
    ; Left sensor value scaled 0-63 on first 2 digits (from left)
    ; and right sensor value scaled 0-63 on last 2 digits
    ; TM1637 keeps the digits itself, no multiplexing needed

    ; data command: write, auto address increment
    rcall tm1637_start
    ldi temp2, TM1637_CMD_DATA
    rcall tm1637_write_byte
    rcall tm1637_stop

    ; address command, then 4 digits from left to right
    rcall tm1637_start
    ldi temp2, TM1637_CMD_ADDR
    rcall tm1637_write_byte

	mov temp2, disp3
	rcall convert_seven_seg
	rcall tm1637_write_byte

	mov temp2, disp2
	rcall convert_seven_seg
	ori temp2, $80          ; add "." / ":" between left and right sensor values
	rcall tm1637_write_byte

	mov temp2, disp1
	rcall convert_seven_seg
	rcall tm1637_write_byte

	mov temp2, disp0
	rcall convert_seven_seg
	rcall tm1637_write_byte
    rcall tm1637_stop

    ; display control: display on, brightness
    rcall tm1637_start
    ldi temp2, TM1637_CMD_DISPLAY
    rcall tm1637_write_byte
    rcall tm1637_stop

	ret
;-------------------------------------------------------------------------
; Convert to Seven Segment (using lookup table)
;-------------------------------------------------------------------------
convert_seven_seg:
	; I changed seven segment subroutine to .db instead of subroutines for each digit
	; convert digit (0-9) in 'temp2' to seven seg value
	ldi		ZL, low(seg_table * 2)	; Load table address into Z pointer
	ldi		ZH, high(seg_table * 2)	; (multiply by 2 for byte addressing)
	add		ZL, temp2				; Add digit offset to table address
	clr		temp
	adc		ZH, temp				; Handle carry if needed
	lpm		temp2, Z				; Load segment pattern from table
	ret

; Seven segment lookup table (0-9)
seg_table:
	.db		$3F, $06	; 0, 1
	.db		$5B, $4F	; 2, 3
	.db		$66, $6D	; 4, 5
	.db		$7D, $07	; 6, 7
	.db		$7F, $6F	; 8, 9
;-------------------------------------------------------------------------
; TM1637 DRIVER (2-wire bit-bang, LSB first, not I2C)
; CLK/DIO are open-drain: sbi DDRB = pull LOW, cbi DDRB = release (HIGH)
; Data may only change while CLK is LOW
;-------------------------------------------------------------------------
tm1637_start:
    ; DIO goes LOW while CLK is HIGH
    cbi DDRB, TM1637_DIO
    cbi DDRB, TM1637_CLK
    rcall tm1637_delay
    sbi DDRB, TM1637_DIO
    rcall tm1637_delay
    ret
;-------------------------------------------------------------------------
tm1637_stop:
    ; DIO goes HIGH while CLK is HIGH
    sbi DDRB, TM1637_CLK
    sbi DDRB, TM1637_DIO
    rcall tm1637_delay
    cbi DDRB, TM1637_CLK
    rcall tm1637_delay
    cbi DDRB, TM1637_DIO
    rcall tm1637_delay
    ret
;-------------------------------------------------------------------------
; write byte in 'temp2' (destroyed), ACK bit is clocked but not checked
tm1637_write_byte:
    ldi bit_count, 8
tm1637_bit_loop:
    sbi DDRB, TM1637_CLK        ; CLK LOW
    lsr temp2                   ; next bit (LSB first) -> carry
    brcc tm1637_bit_zero
    cbi DDRB, TM1637_DIO        ; bit = 1, release DIO
    rjmp tm1637_bit_clock
tm1637_bit_zero:
    sbi DDRB, TM1637_DIO        ; bit = 0, pull DIO LOW
tm1637_bit_clock:
    rcall tm1637_delay
    cbi DDRB, TM1637_CLK        ; CLK HIGH, TM1637 samples DIO
    rcall tm1637_delay
    dec bit_count
    brne tm1637_bit_loop

    ; ACK: release DIO, TM1637 pulls it LOW during 9th clock
    sbi DDRB, TM1637_CLK
    cbi DDRB, TM1637_DIO
    rcall tm1637_delay
    cbi DDRB, TM1637_CLK
    rcall tm1637_delay
    sbi DDRB, TM1637_CLK        ; leave CLK LOW for next byte / stop
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: TM1637_DELAY
; Purpose: ~5us half bit period at 16MHz
;-------------------------------------------------------------------------
; Delay subroutine : 1 + 25*3 - 1 + 4 (ret) + 3 (rcall) = 82 cycles = 5.1 us
tm1637_delay:
    ldi temp, 25
tm1637_delay_loop:
    dec temp
    brne tm1637_delay_loop
    ret
;-------------------------------------------------------------------------
; TIMER0 OVERFLOW INTERRUPT SERVICE ROUTINE (~976Hz, every 1.024ms)
; Purpose: 1) Soft start - PWM rises by 1 per tick toward motor_right /
;             motor_left (0 -> 192 in ~200ms), drops to a lower target at once.
;             A full-PWM start draws a current spike that disturbs the Uno.
;          2) Count down button lockouts and re-enable INT0/INT1
;-------------------------------------------------------------------------
TIM0_OVF:
    push temp
    in temp, SREG
    push temp

    ; right motor ramp (OCR0A)
    in temp, OCR0A
    cp temp, motor_right
    breq ramp_right_done
    brlo ramp_right_up
    mov temp, motor_right       ; above target -> slow down immediately
    rjmp ramp_right_store
ramp_right_up:
    inc temp
ramp_right_store:
    out OCR0A, temp
ramp_right_done:

    ; left motor ramp (OCR2A)
    lds temp, OCR2A
    cp temp, motor_left
    breq ramp_left_done
    brlo ramp_left_up
    mov temp, motor_left        ; above target -> slow down immediately
    rjmp ramp_left_store
ramp_left_up:
    inc temp
ramp_left_store:
    sts OCR2A, temp
ramp_left_done:

    ; Start/Stop button lockout
    lds temp, lock_start
    tst temp
    breq lock_start_done
    dec temp
    sts lock_start, temp
    brne lock_start_done
    ldi temp, (1<<INTF0)        ; drop edges latched during lockout
    out EIFR, temp
    sbi EIMSK, INT0
lock_start_done:

    ; Mode button lockout
    lds temp, lock_mode
    tst temp
    breq lock_mode_done
    dec temp
    sts lock_mode, temp
    brne lock_mode_done
    ldi temp, (1<<INTF1)
    out EIFR, temp
    sbi EIMSK, INT1
lock_mode_done:

    pop temp
    out SREG, temp
    pop temp
    reti
;-------------------------------------------------------------------------
; TIMER2 OVERFLOW INTERRUPT SERVICE ROUTINE
; Purpose: Can be used for timing or additional motor control
;-------------------------------------------------------------------------
TIM2_OVF:
    push temp
    in temp, SREG
    push temp

    ; Timer2 overflow handler code here (if needed)
    ; Currently PWM is handled automatically by hardware

    pop temp
    out SREG, temp
    pop temp
    reti
;-------------------------------------------------------------------------
; INT0 INTERRUPT SERVICE ROUTINE
; Purpose: Toggle robot_state (Start/Stop)
; PD2 (D2) button pressed -> toggle between running (0xFF) and stopped (0x00)
; Motor noise and contact bounce put short LOW spikes on the button line.
; A real press holds PD2 LOW, so the press is accepted only after 10ms of
; continuous LOW, then the button is ignored for ~256ms.
;-------------------------------------------------------------------------
INT0_ISR:
    push temp
    in temp, SREG
    push temp
    push temp2

    ldi temp2, btn_confirm
int0_confirm_outer:
    clr temp                    ; 256 x 5 cycles = 80us
int0_confirm_inner:
    sbic PIND, BTN_START        ; pin went HIGH -> noise, not a press
    rjmp end_robot_state
    dec temp
    brne int0_confirm_inner
    dec temp2
    brne int0_confirm_outer

    ; lock the button, TIM0_OVF re-enables INT0
    cbi EIMSK, INT0
    ldi temp, btn_lockout
    sts lock_start, temp

    ; Toggle robot_state
    com robot_state         ; Complement: 0x00 -> 0xFF, 0xFF -> 0x00

    cpi robot_state, 0xFF
    breq end_robot_state
stop_motors:
    ldi motor_right, motor_off
    ldi motor_left, motor_off

end_robot_state:

    pop temp2
    pop temp
    out SREG, temp
    pop temp
    reti
;-------------------------------------------------------------------------
; INT1 INTERRUPT SERVICE ROUTINE
; Purpose: Toggle dark_mode (Bright/Dark mode)
; PD3 (D3) button pressed -> toggle between bright (0xFF) and dark (0x00) mode
; Same noise filter and lockout as INT0_ISR
;-------------------------------------------------------------------------
INT1_ISR:
    push temp
    in temp, SREG
    push temp
    push temp2

    ldi temp2, btn_confirm
int1_confirm_outer:
    clr temp                    ; 256 x 5 cycles = 80us
int1_confirm_inner:
    sbic PIND, BTN_MODE         ; pin went HIGH -> noise, not a press
    rjmp end_set_threshold
    dec temp
    brne int1_confirm_inner
    dec temp2
    brne int1_confirm_outer

    ; lock the button, TIM0_OVF re-enables INT1
    cbi EIMSK, INT1
    ldi temp, btn_lockout
    sts lock_mode, temp

    ; Toggle dark_mode
    com dark_mode           ; Complement: 0x00 -> 0xFF, 0xFF -> 0x00

    cpi dark_mode, 0x00
    breq set_dark_threshold
    ldi threshold, bright_threshold
    rjmp end_set_threshold
set_dark_threshold:
    ldi threshold, dark_threshold

end_set_threshold:

    pop temp2
    pop temp
    out SREG, temp
    pop temp
    reti
;-------------------------------------------------------------------------
; END OF PROGRAM
;-------------------------------------------------------------------------

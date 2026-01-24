;
; Project.asm
;
; Created: 17/12/2025 20:38:29
; Author : ates1
;
;-------------------------------------------------------------------------
; REGISTER DEFINITIONS
;-------------------------------------------------------------------------
.def temp = R16
.def temp2 = R17
.def sensor_left = R18
.def sensor_right = R19
.def motor_left = R20
.def motor_right = R21
.def robot_state = R22	; $00 close / $FF open
.def dark_mode = R23	; $00 dark  / $FF bright
.def disp0 = R24
.def disp1 = R25
.def disp2 = R26
.def disp3 = R27
.def threshold = R28
;-------------------------------------------------------------------------
; CONSTANTS
;-------------------------------------------------------------------------
; 140 , 140 + 20
.equ motor_on  = 144
.equ full_speed = 144
.equ turn_speed = 128
.equ motor_off = 0
.equ compare_threshold = 10
.equ dark_threshold = 128
.equ bright_threshold = 192
.equ B_controller = 16
;-------------------------------------------------------------------------
; L298N MOTOR DRIVER PIN DEFINITIONS
;-------------------------------------------------------------------------
; Motor A (Right Motor) - PORTB
.equ L298N_ENA  = 3         ; PB3 - ENA (PWM via OC0) - Right Motor
.equ L298N_IN1  = 0         ; PB0 - IN1 (Right Motor Direction)
.equ L298N_IN2  = 1         ; PB1 - IN2 (Right Motor Direction)

; Motor B (Left Motor) - PORTD
.equ L298N_ENB  = 7         ; PD7 - ENB (PWM via OC2) - Left Motor
.equ L298N_IN3  = 2         ; PB2 - IN3 (Left Motor Direction)
.equ L298N_IN4  = 4         ; PB4 - IN4 (Left Motor Direction)


.cseg
.org 0x000
    rjmp RESET

.org 0x002
    jmp INT0_ISR          ; INT0 - Start/Stop Button (PD2)

.org 0x004
    jmp INT1_ISR          ; INT1 - Dark/Bright Mode Button (PD3)

.org 0x00A
    jmp tim2_ovf

.org 0x016
    jmp tim0_ovf

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
    rcall INIT_DISP

	ldi threshold, bright_threshold

	; initialize robot state (0xFF = running, 0x00 = stopped)
	ldi robot_state, 0x00
	; initialize dark mode (0xFF = bright mode, 0x00 = dark mode)
	ldi dark_mode, 0xFF
	sei
;-------------------------------------------------------------------------
; RESET
;-------------------------------------------------------------------------
MAIN_LOOP:
	; read button / PD2, PD3 interrupt button read

	; display sensor data on seven segment
	rcall display_sensor_data

    cpi robot_state, $00
    breq MAIN_LOOP         ; If stopped, skip rest of loop
    
    ; read right sensor
	rcall read_right_sensor

    ; read left sensor	
	rcall read_left_sensor

    ; decision making
	rcall compare_sensors

	; update PWM
	rcall update_motors

    ; update display
    rcall sensor_to_display_values

    rjmp MAIN_LOOP

;-------------------------------------------------------------------------
; INIT SUBROUTINES
;-------------------------------------------------------------------------
INIT_PORTS:
    ; PORTC - Seven Segment Display
    ldi temp, 0xFF
    out PORTC, temp         ; Clear PORTC
    ; PORTA - ADC Input
    ; PA6 = Right LDR sensor (ADC6)
    ; PA7 = Left LDR sensor (ADC7)
    ldi temp, 0x3F
    out DDRA, temp          ; Set PA0-PA3 as outputs (for seven segment digit select)
                            ; PA6-PA7 as inputs (ADC), PA4-PA5 as inputs button-read
    ldi temp, 0x00
    out PORTA, temp         ; No pull-ups (external circuit)
	  
	; PORTB - L298N Motor Driver Pins
    ; PB3 = OC0 (ENA - Right Motor PWM)
    ; PB0 = IN1 (Right Motor Direction)
    ; PB1 = IN2 (Right Motor Direction)
    ; PB2 = IN3 (Left Motor Direction)
    ; PB4 = IN4 (Left Motor Direction)
    ldi temp, (1<<L298N_ENA) | (1<<L298N_IN1) | (1<<L298N_IN2) | (1<<L298N_IN3) | (1<<L298N_IN4)
    out DDRB, temp          ; Set as outputs
	
	ldi temp, (1<<L298N_IN1) | (1<<L298N_IN3)  ; IN1=HIGH, IN3=HIGH (ileri)
	out PORTB, temp

	; PORTD - Left Motor PWM + Button Inputs
	; PD7 = OC2 (ENB - Left Motor PWM)
	; PD2 = INT0 (Start/Stop Button) - Input with pull-up
	; PD3 = INT1 (Dark/Bright Mode Button) - Input with pull-up
    ldi temp, (1<<L298N_ENB)
    out DDRD, temp          ; PD7 as output, PD2/PD3 as inputs (default 0)
	
	; Enable internal pull-ups for buttons (active-low buttons)
	ldi temp, (1<<2) | (1<<3)
	out PORTD, temp

	ret

INIT_ADC:
    ; ADMUX: AVCC reference, Left Adjust Result (8-bit in ADCH)
    ldi temp, (1<<REFS0) | (1<<ADLAR)
    out ADMUX, temp

    ; ADCSRA: Enable ADC, Prescaler = 64 (8MHz/64 = 125kHz ADC clock)
    ldi temp, (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1)
    out ADCSRA, temp

	ret

INIT_PWM:
    ; TIMER0 SETUP - Right Motor PWM (OC0 = PB3)
    ; TCCR0: Fast PWM, Non-Inverting, Prescaler = 64
	ldi temp, (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01) | (1<<CS00)
    out TCCR0, temp
    ; Initialize duty cycle to 0
    ldi temp, 0
    out OCR0, temp

    ; TIMER2 SETUP - Left Motor PWM (OC2 = PD7)
    ; TCCR2: Fast PWM, Non-Inverting, Prescaler = 64
    ldi temp, (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS22)
    out TCCR2, temp
    ; Initialize duty cycle to 0
    ldi temp, 0
    out OCR2, temp

    ret

;-------------------------------------------------------------------------
; INIT_INTERRUPTS
; Purpose: Configure INT0 (PD2) and INT1 (PD3) for button inputs
;          Falling edge triggered (active-low buttons with pull-ups)
;-------------------------------------------------------------------------
INIT_INTERRUPTS:
    ; MCUCR: Set INT0 and INT1 to falling edge trigger
    ; ISC01=1, ISC00=0 -> INT0 falling edge
    ; ISC11=1, ISC10=0 -> INT1 falling edge
    ldi temp, (1<<ISC01) | (1<<ISC11)
    out MCUCR, temp

    ; GICR: Enable INT0 and INT1
    ldi temp, (1<<INT0) | (1<<INT1)
    out GICR, temp

    ret

;-------------------------------------------------------------------------
; INIT_DISP
;-------------------------------------------------------------------------
INIT_DISP:
    ldi disp0, 9
    ldi disp1, 9   
    ldi disp2, 9
    ldi disp3, 9
    ret
/*
SET_MOTORS:
    ; Left Motor Forward: IN1=HIGH, IN2=LOW
    sbi PORTB, L298N_IN1
    cbi PORTB, L298N_IN2
    
    ; Right Motor Forward: IN3=HIGH, IN4=LOW
    sbi PORTB, L298N_IN3
    cbi PORTB, L298N_IN4

    ret
*/

;-------------------------------------------------------------------------
; SUBROUTINE: READ_RIGHT_SENSOR
; Purpose: Read ADC Channel 6 (PA6 - Right LDR) and store in sensor_right
;-------------------------------------------------------------------------
read_right_sensor:
    ; Select ADC Channel 6 (PA6 - Right Sensor)
    ldi temp, (1<<REFS0) | (1<<ADLAR) | 0x06
    out ADMUX, temp

    ; Start conversion
    sbi ADCSRA, ADSC

read_right_wait:
    ; Wait for conversion to complete
    sbic ADCSRA, ADSC
    rjmp read_right_wait

    ; Read result (8-bit from ADCH due to Left Adjust)
    in sensor_right, ADCH

    ret 

;-------------------------------------------------------------------------
; SUBROUTINE: READ_LEFT_SENSOR
; Purpose: Read ADC Channel 7 (PA7 - Left LDR) and store in sensor_left
;-------------------------------------------------------------------------
read_left_sensor:
    ; Select ADC Channel 7 (PA7 - Left Sensor)
    ldi temp, (1<<REFS0) | (1<<ADLAR) | 0x07
    out ADMUX, temp

    ; Start conversion
    sbi ADCSRA, ADSC

read_left_wait:
    ; Wait for conversion to complete
    sbic ADCSRA, ADSC
    rjmp read_left_wait

    ; Read result (8-bit from ADCH due to Left Adjust)
    in sensor_left, ADCH

    ret 

;-------------------------------------------------------------------------
; SUBROUTINE: COMPARE_SENSORS
;-------------------------------------------------------------------------
compare_sensors:
    ; Compare sensor values and set motor speeds accordingly
    cpi robot_state, $00
    breq zero_zero ; If stopped, turn off motors
	cp sensor_right, threshold
	brlo right_0
right_1:
	cp sensor_left, threshold
	brlo one_zero
one_one:
	ldi motor_right, motor_on + B_controller
	ldi motor_left, motor_on 
	ret
right_0:
	cp sensor_left, threshold
	brlo zero_zero
zero_one:
	ldi motor_right, motor_on + B_controller + B_controller
	ldi motor_left, motor_off 
	ret
one_zero:
	ldi motor_right, motor_off
	ldi motor_left, motor_on + B_controller
	ret	
zero_zero:
	ldi motor_right, motor_off
	ldi motor_left, motor_off
	ret

;-------------------------------------------------------------------------
; SUBROUTINE: UPDATE_MOTORS
; Purpose: Apply motor speed values to PWM registers
;-------------------------------------------------------------------------
update_motors:
    ; Update Timer0 OCR for right motor (PB3)
    out OCR0, motor_right

    ; Update Timer2 OCR for left motor (PD7)
    out OCR2, motor_left
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: SEnSOR_TO_DISPLAY_VALUES
;-------------------------------------------------------------------------
sensor_to_display_values:
    cpi robot_state, $00
    breq sensors_zero
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
sensors_zero:
    rcall INIT_DISP
    ret

;-------------------------------------------------------------------------
; SUBROUTINE: DISPLAY_SENSOR_DATA
;-------------------------------------------------------------------------
display_sensor_data:
    ; For demonstration puposes, display sensor data on seven segment
    ; That function print the right sensor value scaled 0-64 on first 2 digit of seven segment
    ; and left sensor value scaled 0-64 on last 2 digit of seven segment
 	
    mov temp2, disp0 
	rcall convert_seven_seg
	ldi temp, $01
	out PORTA, temp
	out PORTC, temp2
	rcall delay1ms

	mov temp2, disp1
	rcall convert_seven_seg
    add temp2, temp
	ldi temp, $02
	out PORTA, temp
	out PORTC, temp2
	rcall delay1ms

	mov temp2, disp2
	rcall convert_seven_seg
	ldi temp , $80	; add "." between second and second/10
    add temp2, temp
	ldi temp, $04
	out PORTA, temp
	out PORTC, temp2
	rcall delay1ms

	mov temp2, disp3
	rcall convert_seven_seg
	ldi temp, $08
	out PORTA, temp
	out PORTC, temp2
	rcall delay1ms

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
; SUBROUTINE: DELAY1MS
; Purpose: Approximate 1ms delay at 8MHzs
;-------------------------------------------------------------------------
; Delay subroutine :  2+256*8*(1+1+2) + (1+1+1) + 4 = 8201 instructions = 1.025 ms
; Delay subroutine :  2+256*4*(1+1+2) + (1+1+1) + 4 = 4105 instructions = 513 ms
delay1ms:		
	ldi		temp, $00
	ldi		temp2, $04
delay:	
	subi	temp, 1  
	sbci	temp2, 0  
	brcc	delay
	ret
;-------------------------------------------------------------------------
; TIMER0 OVERFLOW INTERRUPT SERVICE ROUTINE
; Purpose: Can be used for timing or additional motor control
;-------------------------------------------------------------------------
TIM0_OVF:
    push temp
    in temp, SREG
    push temp

    ; Timer0 overflow handler code here (if needed)
    ; Currently PWM is handled automatically by hardware

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
; PD2 button pressed -> toggle between running (0xFF) and stopped (0x00)
;-------------------------------------------------------------------------
INT0_ISR:
    push temp
    in temp, SREG
    push temp

    ; Toggle robot_state
    com robot_state         ; Complement: 0x00 -> 0xFF, 0xFF -> 0x00
    
    cpi robot_state, 0xFF
    breq end_robot_state
stop_motors:
    ldi motor_right, motor_off
    ldi motor_left, motor_off
    ldi disp0, 9    
    ldi disp1, 9
    ldi disp2, 9
    ldi disp3, 9

end_robot_state:
    ; Simple debounce delay
    ; rcall debounce_delay
    pop temp
    out SREG, temp
    pop temp
    reti

;-------------------------------------------------------------------------
; INT1 INTERRUPT SERVICE ROUTINE  
; Purpose: Toggle dark_mode (Bright/Dark mode)
; PD3 button pressed -> toggle between bright (0xFF) and dark (0x00) mode
;-------------------------------------------------------------------------
INT1_ISR:
    push temp
    in temp, SREG
    push temp

    ; Toggle dark_mode
    com dark_mode           ; Complement: 0x00 -> 0xFF, 0xFF -> 0x00

    cpi dark_mode, 0x00
    breq set_dark_threshold
    ldi threshold, bright_threshold
    rjmp end_set_threshold
set_dark_threshold:
    ldi threshold, dark_threshold

end_set_threshold:
    ; Simple debounce delay
    rcall debounce_delay

    pop temp
    out SREG, temp
    pop temp
    reti

;-------------------------------------------------------------------------
; DEBOUNCE DELAY
; Purpose: ~50ms delay for button debouncing
;-------------------------------------------------------------------------
debounce_delay:
    push temp
    push temp2
    ldi temp, 0x00
    ldi temp2, 0x28         ; ~50ms at 8MHz
debounce_loop:
    subi temp, 1
    sbci temp2, 0
    brcc debounce_loop
    pop temp2
    pop temp
    ret

;-------------------------------------------------------------------------
; END OF PROGRAM
;-------------------------------------------------------------------------
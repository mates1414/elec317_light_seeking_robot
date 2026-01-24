;-------------------------------------------------------------------------
; File Name: main_tank_turn_direct_control.asm
; Created: 17/12/2025 20:38:29
; Author : ates1
; Device : ATmega32
; Crystal: 8MHz
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
;-------------------------------------------------------------------------
; CONSTANTS
;-------------------------------------------------------------------------
.equ motor_turn  = 96       ; PWM value for turning
.equ motor_on  = 192        ; PWM value for moving forward
.equ motor_off = 0          ; PWM value for stopping
.equ compare_threshold = 24 ; Threshold difference between sensors to trigger turn
.equ dark_threshold = 32    ; Light threshold for dark mode
.equ bright_threshold = 160 ; Light threshold for bright mode
.equ B_controller = 16      ; PORT B control constant. 
; L298N motor driver can not give same speed to both ports when both of them working.
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
    ; rcall INIT_DISP

	ldi threshold, bright_threshold 

	; initialize robot state (0xFF = running, 0x00 = stopped)
	ldi robot_state, $00
	; initialize dark mode (0xFF = bright mode, 0x00 = dark mode)
	ldi dark_mode, $FF
    
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

	; update PWM values / motor speeds
	rcall update_motors

    rjmp MAIN_LOOP
;-------------------------------------------------------------------------
open_led_bright:
    ; open led on PD1 if robot is in bright mode
    cpi dark_mode, $00
    breq close_led_bright
    sbi PORTD, PD1
    rjmp end_led_bright
close_led_bright:
    cbi PORTD, PD1
end_led_bright:
    ret 
;-------------------------------------------------------------------------
open_led_run:
    ; open led on PD0 if robot is running
    cpi robot_state, $00
    breq close_led_run
    sbi PORTD, PD0
    rjmp end_led_run
close_led_run:
    cbi PORTD, PD0
end_led_run:
    ret 
;-------------------------------------------------------------------------
; INIT SUBROUTINES
;-------------------------------------------------------------------------
INIT_PORTS:
    ; PORTC - Seven Segment Display
    ldi temp, $FF
    out PORTC, temp         ; 

    ; PORTA - ADC Input
    ; PA6 = Right LDR sensor (ADC6)
    ; PA7 = Left LDR sensor (ADC7)
    ldi temp, $3F
    out DDRA, temp          ; Set PA0-PA3 as outputs (for seven segment digit select)
                            ; PA6-PA7 as inputs (ADC)
    ldi temp, $00
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

	; PORTD - Left Motor PWM + Button Inputs + Status LEDs
	; PD7 = OC2 (ENB - Left Motor PWM)
	; PD2 = INT0 (Start/Stop Button) 
	; PD3 = INT1 (Dark/Bright Mode Button)
	; PD0, PD1 = Status LEDs
    ldi temp, (1<<L298N_ENB) | (1<<PD0) | (1<<PD1)
    out DDRD, temp          ; PD7, PD0, PD1 as outputs, PD2/PD3 as inputs
	
	; Enable pull-ups for buttons, LEDs off initially
	ldi temp, (1<<0) | (1<<1)
	out PORTD, temp
	ret
;-------------------------------------------------------------------------
INIT_ADC:
    ; ADMUX: AVCC reference, Left Adjust Result (8-bit in ADCH)
    ldi temp, (1<<REFS0) | (1<<ADLAR)
    out ADMUX, temp

    ; ADCSRA: Enable ADC, Prescaler = 64 (8MHz/64 = 125kHz ADC clock)
    ldi temp, (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1)
    out ADCSRA, temp

	ret
;-------------------------------------------------------------------------
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

    ; Clear any pending interrupt flags (write 1 to clear)
    ldi temp, (1<<INTF0) | (1<<INTF1)
    out GIFR, temp

    ; GICR: Enable INT0 and INT1
    ldi temp, (1<<INT0) | (1<<INT1)
    out GICR, temp

    ret
;-------------------------------------------------------------------------
; INIT_DISP
;-------------------------------------------------------------------------
/*
INIT_DISP:
    ldi disp0, 9
    ldi disp1, 9   
    ldi disp2, 9
    ldi disp3, 9
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
; Right Motor: IN1=HIGH, IN2=LOW
; Left Motor:  IN3=HIGH, IN4=LOW
;-------------------------------------------------------------------------
set_motors_forward:
    sbi PORTB, L298N_IN1
    cbi PORTB, L298N_IN2
    sbi PORTB, L298N_IN3
    cbi PORTB, L298N_IN4

    ldi motor_right, motor_on
    ldi motor_left, motor_on + B_controller
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: SET_MOTORS_TURN
; Purpose: Set both motors to forward direction
; Right Motor: IN1=HIGH, IN2=LOW
; Left Motor:  IN3=HIGH, IN4=LOW
;-------------------------------------------------------------------------
set_motors_turn:
    sbi PORTB, L298N_IN1
    cbi PORTB, L298N_IN2
    sbi PORTB, L298N_IN3
    cbi PORTB, L298N_IN4
    ; proportional control
    rcall sensor_to_motor   ; adjust sensor values to motor pwm values so they can initiate the movement
    mov motor_right, temp2  ; right sensor value -> left motor speed
	mov motor_left, temp    ; right sensor value -> left motor speed
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: SET_TANK_TURN_LEFT
; Purpose: Tank turn left - Right forward, Left backward
; Right Motor: IN1=HIGH, IN2=LOW  (forward)
; Left Motor:  IN3=LOW,  IN4=HIGH (backward)
;-------------------------------------------------------------------------
set_tank_turn_left:
    sbi PORTB, L298N_IN1
    cbi PORTB, L298N_IN2
    cbi PORTB, L298N_IN3
    sbi PORTB, L298N_IN4

    ldi motor_right, motor_turn
	ldi motor_left, motor_turn + B_controller 
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: SET_TANK_TURN_RIGHT
; Purpose: Tank turn right - Left forward, Right backward
; Right Motor: IN1=LOW,  IN2=HIGH (backward)
; Left Motor:  IN3=HIGH, IN4=LOW  (forward)
;-------------------------------------------------------------------------
set_tank_turn_right:
    cbi PORTB, L298N_IN1
    sbi PORTB, L298N_IN2
    sbi PORTB, L298N_IN3
    cbi PORTB, L298N_IN4
	
    ldi motor_right, motor_turn
	ldi motor_left, motor_turn + B_controller
    ret
;-------------------------------------------------------------------------
; SUBROUTINE: SET_MOTORS_STOP
; Purpose: Stop both motors (brake mode)
; Both IN pins LOW = coast, Both HIGH = brake
;-------------------------------------------------------------------------
set_motors_stop:
    cbi PORTB, L298N_IN1
    cbi PORTB, L298N_IN2
    cbi PORTB, L298N_IN3
    cbi PORTB, L298N_IN4

	ldi motor_right, motor_off
	ldi motor_left, motor_off
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
	ldi temp , $80	; add "." between left and right sensor values
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
; Delay subroutine :  2+256*4*(1+1+2) + (1+1+1) + 4 = 4105 instructions = 0.513 ms
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

end_robot_state:

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

    pop temp
    out SREG, temp
    pop temp
    reti
;-------------------------------------------------------------------------
; END OF PROGRAM
;-------------------------------------------------------------------------
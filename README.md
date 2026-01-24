# Light Following Robot with Tank Turn Control

A line/light following robot project built with ATmega32 microcontroller, featuring tank turn capability and proportional control for smooth navigation.

## Project Overview

This project implements an autonomous light-following robot that uses two LDR (Light Dependent Resistor) sensors to detect light sources and navigate towards them. The robot features tank-turn capability for sharp turns and proportional control for smooth tracking.

### Key Features

- **Tank Turn Mechanism**: Enables the robot to turn in place by rotating wheels in opposite directions
- **Proportional Control**: Adjusts motor speeds based on sensor readings for smooth navigation
- **Dual Mode Operation**: Switch between bright and dark mode with different light thresholds
- **Start/Stop Control**: External button to toggle robot operation
- **Real-time Display**: Seven-segment display showing sensor values
- **LED Status Indicators**: Visual feedback for operating mode and running state

## Hardware Design

### Microcontroller

| Component | Specification |
|-----------|--------------|
| MCU | ATmega32 |
| Crystal | 8 MHz |
| Development Board | EasyAVR7 (or compatible) |

### Component List

| Component | Quantity | Description |
|-----------|----------|-------------|
| ATmega32 | 1 | Main microcontroller |
| L298N Motor Driver | 1 | Dual H-Bridge motor driver module |
| DC Motors | 2 | Geared DC motors for tank drive |
| LDR Sensors | 2 | Light Dependent Resistors for light detection |
| 10kΩ Resistors | 2 | Voltage divider for LDR circuit |
| 4-Digit 7-Segment Display | 1 | Common cathode, for sensor value display |
| Push Buttons | 2 | Start/Stop and Mode selection |
| LEDs | 2 | Status indicator LEDs |
| 330Ω Resistors | 2 | Current limiting for LEDs |
| Robot Chassis | 1 | Tank-style chassis with wheels |
| Power Supply | 1 | Battery pack (6-12V for motors) |
| Jumper Wires | - | For connections |

### Pin Configuration

#### PORTA - ADC & Display Control
| Pin | Function |
|-----|----------|
| PA0-PA3 | Seven segment digit select (outputs) |
| PA6 | Right LDR sensor (ADC6 input) |
| PA7 | Left LDR sensor (ADC7 input) |

#### PORTB - Motor Driver Control
| Pin | Function |
|-----|----------|
| PB0 | L298N IN1 (Right motor direction) |
| PB1 | L298N IN2 (Right motor direction) |
| PB2 | L298N IN3 (Left motor direction) |
| PB3 | L298N ENA - OC0 (Right motor PWM) |
| PB4 | L298N IN4 (Left motor direction) |

#### PORTC - Seven Segment Display
| Pin | Function |
|-----|----------|
| PC0-PC7 | Seven segment data lines |

#### PORTD - PWM, Buttons & LEDs
| Pin | Function |
|-----|----------|
| PD0 | Running status LED |
| PD1 | Bright mode status LED |
| PD2 | INT0 - Start/Stop button (active low) |
| PD3 | INT1 - Dark/Bright mode button (active low) |
| PD7 | L298N ENB - OC2 (Left motor PWM) |

### L298N Motor Driver Connections

```
ATmega32          L298N Module
---------         ------------
PB3 (OC0)   -->   ENA (Right Motor Speed)
PB0         -->   IN1 (Right Motor Direction)
PB1         -->   IN2 (Right Motor Direction)
PB2         -->   IN3 (Left Motor Direction)
PB4         -->   IN4 (Left Motor Direction)
PD7 (OC2)   -->   ENB (Left Motor Speed)
GND         -->   GND
```

### LDR Sensor Circuit

```
VCC (5V)
   |
  [LDR]
   |
   +-----> To ADC Pin (PA6/PA7)
   |
 [10kΩ]
   |
  GND
```

## Circuit Diagram

```
                    +------------------+
                    |     ATmega32     |
                    |                  |
  Right LDR ------->| PA6         PB3 |-------> L298N ENA
   Left LDR ------->| PA7         PB0 |-------> L298N IN1
                    |             PB1 |-------> L298N IN2
  7-Seg Data <------| PC0-PC7     PB2 |-------> L298N IN3
  7-Seg Sel  <------| PA0-PA3     PB4 |-------> L298N IN4
                    |                  |
   Run LED <--------| PD0         PD7 |-------> L298N ENB
  Mode LED <--------| PD1              |
Start/Stop -------->| PD2 (INT0)       |
 Mode Btn  -------->| PD3 (INT1)       |
                    +------------------+
```

## Software Architecture

### Main Program Flow

1. **Initialization**: Setup ports, ADC, PWM timers, and external interrupts
2. **Main Loop**:
   - Update status LEDs
   - Read right LDR sensor (ADC6)
   - Read left LDR sensor (ADC7)
   - Compare sensors and make navigation decision
   - Convert sensor values for display
   - Display data on seven-segment
   - Update motor PWM values

### PWM Configuration

| Timer | Output Pin | Function | Mode |
|-------|------------|----------|------|
| Timer0 | PB3 (OC0) | Right motor speed | Fast PWM, Prescaler 64 |
| Timer2 | PD7 (OC2) | Left motor speed | Fast PWM, Prescaler 64 |

### Motor Control Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `motor_on` | 192 | PWM value for forward movement (~75% duty) |
| `motor_turn` | 96 | PWM value for turning (~37.5% duty) |
| `motor_off` | 0 | PWM value for stop |
| `compare_threshold` | 24 | Sensor difference threshold for turn decision |
| `bright_threshold` | 160 | Light detection threshold in bright mode |
| `dark_threshold` | 32 | Light detection threshold in dark mode |
| `B_controller` | 16 | Compensation for motor speed difference |

### Navigation Logic

The robot uses a decision tree based on sensor readings:

| Right Sensor | Left Sensor | Action |
|--------------|-------------|--------|
| < threshold | < threshold | Stop (no light detected) |
| ≥ threshold | ≥ threshold | Go forward (proportional adjustment if difference exists) |
| ≥ threshold | < threshold | Tank turn right |
| < threshold | ≥ threshold | Tank turn left |

### Tank Turn Implementation

Tank turns allow the robot to rotate in place:
- **Turn Right**: Left motor forward, Right motor backward
- **Turn Left**: Right motor forward, Left motor backward

## Operating Modes

### Bright Mode (Default)
- Threshold: 160
- Suitable for well-lit environments
- LED on PD1 is ON

### Dark Mode
- Threshold: 32
- Suitable for low-light environments
- LED on PD1 is OFF

Toggle between modes using the button connected to PD3 (INT1).

## Button Controls

| Button | Pin | Function |
|--------|-----|----------|
| Start/Stop | PD2 (INT0) | Toggle robot running state |
| Mode | PD3 (INT1) | Toggle between Bright/Dark mode |

## Seven Segment Display

The display shows sensor readings scaled to 0-63:
- **Digits 0-1**: Right sensor value
- **Digits 2-3**: Left sensor value
- A decimal point separates left and right values

## Files in Project

| File | Description |
|------|-------------|
| `main_tank_turn_direct_control.asm` | **Main program** - Latest version with tank turn and direct control |
| `main_tank_turn.asm` | Tank turn version (earlier) |
| `main_direct_control.asm` | Direct control version without tank turn |
| `main.asm` | Basic version |
| `pwm_motor_controller.asm` | PWM motor control testing/development |
| `pwm_volt.asm` | PWM voltage control experiments |
| `ldr.asm` | LDR sensor testing code |
| `deneme.asm` | Test/experiment file |

## Building and Flashing

1. Open the project in Atmel Studio or compatible AVR IDE
2. Select ATmega32 as target device
3. Set crystal frequency to 8MHz
4. Build the project
5. Flash `main_tank_turn_direct_control.asm` to the microcontroller

## Calibration

1. Power on the robot in a controlled lighting environment
2. Observe sensor values on the seven-segment display
3. Adjust `bright_threshold` and `dark_threshold` constants if needed
4. Modify `compare_threshold` to fine-tune turn sensitivity
5. Adjust `B_controller` if motors have different speeds

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Robot doesn't move | Robot in stopped state | Press Start button (PD2) |
| Robot always turns one direction | Unbalanced sensor readings | Adjust sensor positions or thresholds |
| Motors have different speeds | Motor driver asymmetry | Increase `B_controller` value |
| Erratic behavior | Noise in ADC readings | Add capacitors to sensor circuit |
| No display output | Wrong digit select | Check PORTA connections |

## Author

- **Author**: ates1
- **Course**: ELEC317
- **Institution**: Koç University
- **Date**: December 2025

## License

This project is developed for educational purposes as part of the ELEC317 course.

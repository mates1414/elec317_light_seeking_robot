# Light Following Robot with Tank Turn Control (Arduino Uno port)

A light following robot project, originally built with ATmega32 and ported to Arduino Uno (ATmega328P), featuring tank turn capability and proportional control for smooth navigation. Still written in AVR assembly.

> The original ATmega32 / EasyAVR7 / L298N version is on the `master` branch.

## Project Overview

This project implements an autonomous light-following robot that uses two LDR (Light Dependent Resistor) sensors to detect light sources and navigate towards them. The robot features tank-turn capability for sharp turns and proportional control for smooth tracking.

### Key Features

- **Tank Turn Mechanism**: Enables the robot to turn in place by rotating wheels in opposite directions
- **Proportional Control**: Adjusts motor speeds based on sensor readings for smooth navigation
- **Dual Mode Operation**: Switch between bright and dark mode with different light thresholds
- **Start/Stop Control**: External button to toggle robot operation
- **Real-time Display**: TM1637 4-digit seven-segment display showing sensor values
- **LED Status Indicators**: Visual feedback for operating mode and running state

## Hardware Design

### Microcontroller

| Component | Specification |
|-----------|--------------|
| MCU | ATmega328P |
| Crystal | 16 MHz |
| Development Board | Arduino Uno |

### Component List

| Component | Quantity | Description |
|-----------|----------|-------------|
| Arduino Uno | 1 | Main microcontroller board |
| TB6612FNG Motor Driver | 1 | Dual H-Bridge motor driver module |
| TT Gear Motors + 66 mm wheels | 2 | Geared DC motors for tank drive |
| LDR Sensors (GL55) | 2 | Light Dependent Resistors for light detection |
| 10kΩ Resistors | 2 | Voltage divider for LDR circuit |
| TM1637 4-Digit 7-Segment Module | 1 | For sensor value display |
| Push Buttons | 2 | Start/Stop and Mode selection |
| LEDs | 2 | Status indicator LEDs |
| 330Ω Resistors | 2 | Current limiting for LEDs |
| 100nF Ceramic Capacitors | 2 | Motor noise suppression |
| 2S 18650 Battery Holder + Switch | 1 | 7.4V (8.4V full) power |
| 3D Printed Chassis | 1 | base_plate, top_plate, uno_adapter, ldr_mount, front_skid |
| Jumper Wires / Mini Breadboard | - | For connections |

### Pin Configuration

| Uno Pin | AVR Port | Function |
|---------|----------|----------|
| D6 | PD6 / OC0A | TB6612 PWMA (Right motor PWM) |
| D7 | PD7 | TB6612 AIN1 (Right motor direction) |
| D8 | PB0 | TB6612 AIN2 (Right motor direction) |
| D11 | PB3 / OC2A | TB6612 PWMB (Left motor PWM) |
| D4 | PD4 | TB6612 BIN1 (Left motor direction) |
| D12 | PB4 | TB6612 BIN2 (Left motor direction) |
| D2 | PD2 / INT0 | Start/Stop button (other leg to GND, internal pull-up) |
| D3 | PD3 / INT1 | Dark/Bright mode button (other leg to GND, internal pull-up) |
| A0 | PC0 / ADC0 | Right LDR sensor |
| A1 | PC1 / ADC1 | Left LDR sensor |
| A2 | PC2 | Running status LED (via 330Ω) |
| A3 | PC3 | Bright mode status LED (via 330Ω) |
| D9 | PB1 | TM1637 CLK |
| D10 | PB2 | TM1637 DIO |

D0/D1 (USB serial) are left free so the board can be programmed over USB. A4/A5 (I2C) and D13 are free for future sensors.

### TB6612FNG Motor Driver Connections

```
Arduino Uno       TB6612FNG
-----------       ---------
D6  (OC0A)  -->   PWMA (Right Motor Speed)
D7          -->   AIN1 (Right Motor Direction)
D8          -->   AIN2 (Right Motor Direction)
D11 (OC2A)  -->   PWMB (Left Motor Speed)
D4          -->   BIN1 (Left Motor Direction)
D12         -->   BIN2 (Left Motor Direction)
5V          -->   VCC, STBY
Battery +   -->   VM (through switch)
GND         -->   GND (all grounds common)

AO1/AO2 --> Right motor, BO1/BO2 --> Left motor
```

### Power

- Battery + → switch → Uno VIN and TB6612 VM. Battery − → GND.
- A full 2S pack gives 8.4V, TT motors are rated 3–6V. Keep the PWM limit around 192/255 (~6.3V).
- Recharge at ~6.5V, the Uno regulator can not hold 5V below that.

### LDR Sensor Circuit

```
VCC (5V)
   |
  [LDR]
   |
   +-----> To ADC Pin (A0/A1)
   |
 [10kΩ]
   |
  GND
```

## Circuit Diagram

```
                    +------------------+
                    |   Arduino Uno    |
                    |                  |
  Right LDR ------->| A0           D6 |-------> TB6612 PWMA
   Left LDR ------->| A1           D7 |-------> TB6612 AIN1
                    |              D8 |-------> TB6612 AIN2
   Run LED <--------| A2          D11 |-------> TB6612 PWMB
  Mode LED <--------| A3           D4 |-------> TB6612 BIN1
                    |             D12 |-------> TB6612 BIN2
Start/Stop -------->| D2 (INT0)        |
 Mode Btn  -------->| D3 (INT1)    D9 |-------> TM1637 CLK
                    |             D10 |<------> TM1637 DIO
                    +------------------+
```

## Software Architecture

### Main Program Flow

1. **Initialization**: Setup ports, ADC, PWM timers, and external interrupts
2. **Main Loop**:
   - Update status LEDs
   - Read right LDR sensor (ADC0)
   - Read left LDR sensor (ADC1)
   - Compare sensors and make navigation decision
   - Convert sensor values for display
   - Send data to TM1637 display
   - Update motor PWM values

### PWM Configuration

| Timer | Output Pin | Function | Mode |
|-------|------------|----------|------|
| Timer0 | D6 (OC0A) | Right motor speed | Fast PWM, Prescaler 64 (~976 Hz) |
| Timer2 | D11 (OC2A) | Left motor speed | Fast PWM, Prescaler 64 (~976 Hz) |

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

### Changes from the ATmega32 version

| Topic | ATmega32 (master) | ATmega328P (arduino) |
|-------|-------------------|----------------------|
| Include | `m32def.inc` | `m328Pdef.inc` |
| Clock | 8 MHz | 16 MHz |
| Timer0 PWM | `TCCR0`, `OCR0` (PB3) | `TCCR0A/TCCR0B`, `OCR0A` (D6) |
| Timer2 PWM | `TCCR2`, `OCR2` (PD7) | `TCCR2A/TCCR2B`, `OCR2A` (D11), via `sts` |
| External interrupts | `GICR`, `MCUCR`, `GIFR` | `EIMSK`, `EICRA` (`sts`), `EIFR` |
| Interrupt vectors | Fixed addresses | `INT0addr`, `INT1addr`, `OVF0addr`, `OVF2addr` |
| ADC | ADC6/ADC7, prescaler 64, `in`/`out`/`sbi` | ADC0/ADC1, prescaler 128, `lds`/`sts` (extended I/O) |
| Motor driver | L298N | TB6612FNG (same IN logic) |
| Display | Multiplexed 7-seg on PORTA/PORTC | TM1637, 2-wire bit-bang driver |
| Buttons | External pull-ups | Internal pull-ups |

## Operating Modes

### Bright Mode (Default)
- Threshold: 160
- Suitable for well-lit environments
- LED on A3 is ON

### Dark Mode
- Threshold: 32
- Suitable for low-light environments
- LED on A3 is OFF

Toggle between modes using the button connected to D3 (INT1).

## Button Controls

| Button | Pin | Function |
|--------|-----|----------|
| Start/Stop | D2 (INT0) | Toggle robot running state |
| Mode | D3 (INT1) | Toggle between Bright/Dark mode |

## Seven Segment Display

The TM1637 display shows sensor readings scaled to 0-63 as `LL:RR`:
- **Left two digits**: Left sensor value
- **Right two digits**: Right sensor value
- The colon (or decimal point, depending on the module) separates left and right values

## Files in Project

| File | Description |
|------|-------------|
| `main_tank_turn_direct_control.asm` | **Main program** - Arduino Uno (ATmega328P) port |
| `main_tank_turn.asm` | Tank turn version (earlier, ATmega32 only) |
| `main_direct_control.asm` | Direct control version without tank turn (ATmega32 only) |
| `main.asm` | Basic version (ATmega32 only) |
| `pwm_motor_controller.asm` | PWM motor control testing/development (ATmega32 only) |
| `pwm_volt.asm` | PWM voltage control experiments (ATmega32 only) |
| `ldr.asm` | LDR sensor testing code (ATmega32 only) |
| `deneme.asm` | Test/experiment file (ATmega32 only) |

## Building and Flashing

### 1. Build the .hex

**Option A - Microchip Studio (Atmel Studio 7)**

1. Open `Project.atsln`
2. The project is already set to ATmega328P and `main_tank_turn_direct_control.asm` as the entry file
3. Build → Build Solution (F7)
4. Output: `Project/Debug/Project.hex`

**Option B - avra (command line)**

```
avra -I C:\path\to\avra\includes main_tank_turn_direct_control.asm
```

Run it inside `Project/`. `-I` points to the folder containing `m328Pdef.inc` (not needed if avra is installed system-wide).

Output: `main_tank_turn_direct_control.hex`. The register alias warnings (r26-r29 already assigned to X/Y) are expected.

### 2. Upload to the Uno

The Uno bootloader is used over USB, no external programmer (USBasp) is needed. `avrdude` comes with the Arduino IDE, or can be installed separately.

1. Connect the Uno with USB and find its COM port (Device Manager → Ports, or Arduino IDE → Tools → Port)
2. Run:

```
avrdude -c arduino -p m328p -P COM3 -b 115200 -U flash:w:Project.hex:i
```

Replace `COM3` with your port (`/dev/ttyACM0` on Linux). Close the Arduino IDE serial monitor before uploading, it holds the port. Some Uno clones use the old bootloader at `-b 57600`.

## Calibration

1. Put the robot on a box with the wheels in the air
2. Drive each motor forward. If one spins backwards, swap its two wires
3. Test tank turn: turning right, left wheel goes forward and right wheel goes backward
4. Hold the light straight ahead, both LDR values on the TM1637 should be close
5. Adjust `B_controller` if motors have different speeds
6. Retune `compare_threshold` and `motor_turn`; wheels are now at the back, so the old values may not fit
7. Adjust `bright_threshold` and `dark_threshold` for your environment

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Robot doesn't move | Robot in stopped state | Press Start button (D2) |
| Robot doesn't move, LED on | TB6612 STBY not connected | Connect STBY to 5V |
| Robot always turns one direction | Unbalanced sensor readings | Adjust sensor positions or thresholds |
| Motors have different speeds | Motor asymmetry | Increase `B_controller` value |
| Erratic behavior / resets | Motor noise | Add 100nF capacitors across motor terminals |
| No display output | Wrong CLK/DIO wiring | Check D9 → CLK, D10 → DIO |
| Upload fails | Wrong port / baud rate | Check COM port, try `-b 57600` |

## Author

- **Author**: ates1
- **Course**: ELEC317
- **Institution**: Koç University
- **Date**: December 2025

## License

This project is developed for educational purposes as part of the ELEC317 course.

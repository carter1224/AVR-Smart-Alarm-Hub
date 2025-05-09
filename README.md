# Smart Alarm & Light Hub

## Overview
This project implements a multifunctional embedded system on an AVR microcontroller, combining:

- **Alarm & Door Lock**: Keypad-controlled arming/disarming with stepper-driven door lock
- **Light Dimmer & Voltage Monitor**: PWM-based LED dimming with real-time voltage display
- **SPI Communication**: Status updates sent to a remote hub

## Features

### 1. Alarm & Door Lock
- **States**: `DISARMED` ↔ `ARMED`
- **User Codes**: Two distinct 3-button sequences (one to arm, one to disarm)
- **Debounce**: Software debounce implemented in a 1 ms Timer 1 ISR (8‑sample window)
- **Countdown**: 10 s countdown (displayed on LCD) before locking; stepper rotates 90° CW
- **Indicators**:
  - *ARMED*: Red LED on, green LED off, LCD shows “ARMED”
  - *ALARM* (wrong code in ARMED): Red LED blinks at 4 Hz, buzzer active, LCD shows “ALARM”
  - *DISARMED*: Stepper rotates 90° CCW, green LED on, red LED off, LCD shows “DISARMED”
- **Timer 1**: CTC mode, OCR1A=15999 → 1 ms tick @16 MHz; ISR handles debouncing, countdown, LED blink timing

### 2. Light Dimmer & Voltage Monitor
- **Toggle**: Pushbutton on INT0 toggles blue LED on/off
- **Dimming**: Potentiometer read via ADC (8 MSB), output via PWM to control LED brightness
- **Voltage Display**: Measured voltage shown on LCD row 1 (last four digits)

### 3. SPI Communication to Remote Hub
- **Trigger**: Send `0xAA` (alarm), `0x55` (disarm) over SPI master
- **Configuration**: CPOL=1, CPHA=1, SCK=2 MHz
- **Interrupt**: SPI interrupt on transmission complete; remote HQ updates its LCD

## Hardware
- **MCU**: ATmega328P @16 MHz (5 V)
- **Stepper**: 28BYJ‑48 + ULN2003 (for door locking)
- **Inputs**: 3 × pushbuttons (code entry), 1 × pushbutton (dimmer), potentiometer
- **Outputs**: 4 × LEDs (red, green, blue, status), active buzzer, I²C LCD (20×4)
- **Communication**: SPI to remote hub

---
*Developed for ECE 484: Embedded Systems Design (Spring 2025)*

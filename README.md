# Smart Alarm & Light Hub Project

## Overview

This repository hosts my Embedded Systems class project: a Smart Alarm & Light Hub integrating an alarm with door lock, a light-dimming interface with voltage monitoring, and SPI-based communication to a remote hub.

## Features

### Alarm System
- Two states: **ARMED** and **DISARMED**, controlled via distinct 3‑button codes entered on pushbuttons.  
- Input code displayed on second LCD row; incorrect sequences clear input before retry.  
- Correct arm code initiates a **10 s countdown** (displayed in last two digits of LCD row 2 via I2C), after which the stepper motor rotates 90° CW to lock the door.  
- Door-locked state: red LED on, green LED off, LCD shows “ARMED.”  
- In ARMED state, an incorrect disarm code triggers the alarm: red LED blinks every 250 ms, active buzzer sounds, LCD shows “ALARM.”  
- Correct disarm code rotates stepper motor 90° CCW to unlock; red LED off, green LED on, LCD displays “DISARMED.”  
- **Timer 1** serves as the system clock; its ISR handles button debouncing and the state‐machine timing.

### Light Dimmer & Voltage Monitor
- Pushbutton on **INT0** toggles blue LED on/off.  
- Potentiometer input (analog channel) quantized to the 8 MSB; converted back to analog voltage.  
- Real‑time voltage shown on first LCD row’s last four digits.

### SPI Communication to Remote Hub
- When the alarm is triggered, send **0b10101010** to the Alarm HQ via SPI master.  
- Upon disarm, send **0b01010101**.  
- SPI configured with **idle clock high (CPOL=1)**, **sample on falling edge (CPHA=1)**, **SCK = 2 MHz**.  
- Transmission complete triggers an SPI interrupt; remote HQ displays status on its LCD.

## Hardware Setup

- **ATmega328P microcontroller** (AVR-based)  
- **Breadboard** and **Power Supply Module** (5 V output)  
- **Jumper Wires** for prototyping connections  
- **28BYJ-48 Stepper Motor** and **ULN2003 Driver Board** (for door lock mechanism)  
- **Active Buzzer** for audible alarm  
- **Discrete LEDs** (various colors) for visual status indication  
- **Tactile Pushbuttons** for user input  
- **Potentiometer** for PWM brightness control (used in the light dimmer interface)

---
*Crafted with passion for embedded systems!*  

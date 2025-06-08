# GROUP9_DESIGN_PROJECT_1

STM32-Based Smart Environment Monitor

Overview

This project is a Smart Environment Monitor built using an STM32 development board and FreeRTOS. It collects environmental data from sensors, displays the readings on an OLED screen, and transmits them over UART to a host terminal. The system demonstrates real-time task scheduling, inter-task communication, and hardware interfacing with multiple peripherals.

Goals

Monitor temperature, humidity, light levels, and optionally pressure
Display data on an I2C OLED screen
Transmit readings via UART to a terminal (e.g., PuTTY)
Enable system interaction using a push button
Manage tasks using FreeRTOS
Indicate system status with a heartbeat LED

Hardware Used

STM32 Nucleo-F401RE (or similar STM32 board)
BME280 Sensor (I2C) — temperature, humidity, pressure
LDR + Resistor — ambient light level (ADC input)
0.96" I2C OLED Display (SSD1306)
Push Button with pull-down resistor
Breadboard and jumper wires
CP2102

Minimum Functionality

SensorReadTask: Reads temperature, humidity, light, and pressure periodically.

DisplayUpdateTask: Updates OLED with live sensor data.

UARTTransmitTask: Sends formatted data strings to the host via UART.

ButtonTask: Handles button press using external interrupt.

StatusTask: Blinks LED to indicate system is running.

Inter-Task Communication
Uses FreeRTOS Queues to safely transfer sensor data between tasks.







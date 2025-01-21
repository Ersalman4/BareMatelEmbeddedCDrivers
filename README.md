# Project: Real Time Clock Using (DS1307) Display Over LCD 16x2
This project demonstrates how to interface the DS1307 Real-Time Clock (RTC) chip with 
an STM32 microcontroller and display the time and date on an LCD.

# Features
- Interface with DS1307 RTC over I2C
- Display time and date on a 16x2 LCD
- Battery backup support for DS1307
- Configurable time settings via STM32 firmware
  
# Hardware Required: stm32f411 nucleo board, rtc_chip(ds1307), lcd(16x2), jumper wires 

# BareMetal Embedded C Drivers

This repository contains bare-metal drivers for STM32 microcontrollers developed using STM32CubeIDE 

# Project Structure
- `Src/`: this is having our main application code 
- `BSP` : header and source files for rtc_chip and lcd.
- `Drivers/`: low level drivers written by me from scratch by reading reference manual 
  - `Src/`: Source files
  - `Inc/`: Header files
- `Debug/`: Build output (ignored in the repository)

# How to Use
1. Clone the repository:
   ```bash
   git clone https://github.com/Ersalman4/BareMatelEmbeddedCDrivers.git

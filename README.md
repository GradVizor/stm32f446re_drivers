# STM32F446RE Peripheral Drivers
This repository contains drivers for accessing various peripherals of the STM32F446RE board, including GPIO, I2C, SPI, USART, and RCC. The drivers are organized into include (inc) and source (src) directories.
# Table of Contents
1) Introduction

2) Directory Structure

3) Getting Started

4) Supported Peripherals

    GPIO
   
    I2C
   
    SPI
   
    USART

    RCC

# Introduction
This repository provides low-level access to the peripherals of the STM32F446RE microcontroller. These drivers are designed to be lightweight and efficient, allowing for direct manipulation of the hardware registers.

# Directory Structure
stm32f446re_drivers/
├── drivers/

│   ├── inc/

│   │   ├── stm32f446re_gpio_driver.h

│   │   ├── stm32f446re_i2c_driver.h

│   │   ├── stm32f446re_rcc_driver.h

│   │   ├── stm32f446re_spi_driver.h

│   │   └── stm32f446re_usart_driver.h

│   │   └── stm32f446re.h

│   └── src/

│       ├── stm32f446re_gpio_driver.c

│       ├── stm32f446re_i2c_driver.c

│       ├── stm32f446re_rcc_driver.c

│       ├── stm32f446re_spi_driver.c

│       └── stm32f446re_usart_driver.c

# Getting Started
1) Clone the repository:-
   
   `git clone https://github.com/your_username/stm32f446re_drivers.git` & then `cd stm32f446re_drivers `
   
2) Include the drivers in your project:-
   
    a) Add the drivers/inc directory to your include path.
   
    b) Add the drivers/src directory to your source files.

3) Compile your project:
   
Ensure your build system includes the necessary source files and header paths. 

# Supported Peripherals

**GPIO**

The GPIO driver allows for the configuration and control of the general-purpose input/output pins.

**I2C**

The I2C driver provides an interface for the inter-integrated circuit communication protocol, enabling communication with I2C-compatible devices.

**SPI**

The SPI driver enables the use of the serial peripheral interface, facilitating communication with SPI devices.

**USART**

The USART driver supports the universal synchronous/asynchronous receiver-transmitter, enabling serial communication.

**RCC**

The RCC driver manages the reset and clock control system, allowing for the configuration of system clocks.


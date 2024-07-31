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

5) Usage

6) License

# Introduction
This repository provides low-level access to the peripherals of the STM32F446RE microcontroller. These drivers are designed to be lightweight and efficient, allowing for direct manipulation of the hardware registers.

# Directory Structure
stm32f446re_drivers/
├── drivers/

│   ├── inc/

│   │   ├── gpio.h

│   │   ├── i2c.h

│   │   ├── spi.h

│   │   ├── usart.h

│   │   └── rcc.h

│   └── src/

│       ├── gpio.c

│       ├── i2c.c

│       ├── spi.c

│       ├── usart.c

│       └── rcc.c

├── examples/

│   ├── gpio_example.c

│   ├── i2c_example.c

│   ├── spi_example.c


│   ├── usart_example.c
│   └── rcc_example.c
└── README.md


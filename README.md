# STM32 Programming with opencm3: Part 2

This repository contains the complete companion code and EAGLE design files
to accompany
[this blog post](https://rhye.org/post/stm32-with-opencm3-2-spi-and-dma)
which works through using SPI and DMA on an STM32 series MCU.

You can also check out the two previous posts in the series:

- [Compiling and Uploading](https://rhye.org/post/stm32-with-opencm3-0-compiling-and-uploading/),
- [Alternate Functions and USARTs](https://rhye.org/post/stm32-with-opencm3-1-usart-and-printf/)

In this repo:
- `main.cpp`: The main example code
- `stm32f0.ld`: Linker script for the `STM32F070CB` MCU
- `Makefile`: Collection of rules to build and upload code to the MCU
- `.gdbinit`: GDB resource file that automates conencting to an attached Black Magic Probe
- `libopencm3`: Git submodule referencing our version of opencm3.

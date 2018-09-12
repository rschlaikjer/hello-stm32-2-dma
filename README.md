# STM32 Programming with opencm3: Part 1

This repository contains the complete companion code and EAGLE design files
to accompany
[this blog post](https://rhye.org/post/stm32-with-opencm3-1-usart-and-printf/)
which details setting up a USART, and redirecting printf to it.

You can also check out
[the previous post](https://rhye.org/post/stm32-with-opencm3-0-compiling-and-uploading/),
which details the basics of compiling and flashing code.

In this repo:
- `main.cpp`: The main example code with USART.
- `stm32f0.ld`: Linker script for the `STM32F070CB` MCU
- `Makefile`: Collection of rules to build and upload code to the MCU
- `.gdbinit`: GDB resource file that automates conencting to an attached Black Magic Probe
- `libopencm3`: Git submodule referencing our version of opencm3.

## Pictures

Console output with formatted timestamp
![Timestamped console output](/img/timestamp_console.png?raw=true)

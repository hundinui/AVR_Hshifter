# AVR_Hshifter
Arduino/AVR based H-Shifter controller

Licenced under GNU GPLv3

Mostly untested, since switched to a STM32 based controller for the project instead.

Setup was that the Arduino's ATmega16U2 (so cheap chinese clones with the CH340 USB to serial won't work) was flashed with the HID firmware that accepted button states over USART.

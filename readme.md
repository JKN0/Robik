# Robik

A Rubik's cube solving robot built from 99% of junk. 

Materials used: 
- two inkjet printers 
- robot vacuum cleaner 
- electricity meter 
- miscellaneous electronic junk
- plastic, metal and wood leftovers

Video is here: https://youtu.be/gjVkjhh7wAA

## Mechanics

Mechanics comes mostly from two inkjet printers (Xerox Docuprint and HP Officejet) and robot vacuum cleaner (Yujin iClebo).

- Horizontal movement is the print head moving mechanism from Xerox
- Cube rotation/twisting is from vacuum cleaner wheel and OfficeJet paper feeder
- Color detector movement is from Xerox print head cleaning mechanism

## Electronics

Because electronics consists of repurposed devices, there exists no actual schematic. A block diagram is in /hw directory.

Device contains two microcontrollers: 
- STM32F101 (vacuum cleaner board): mechanics and display control
- LPC2366 (electricity meter board): cube solving algorithm

Xerox stepper motor controllers are used for horizontal movement (tilting) and color detector motors. 
Twister DC-motor is controlled directly from vacuum cleaner board.

Vacuum board and meter board communicate via serial port.

## Software

/robik_main
- vacuum cleaner board software
- mechanics control
- runs under FreeRTOS

/solver
- electricity meter board software
- cube solving and color detection error correction
- uses Thistlewaite's algorithm by Jaap Scherphuis
- no operating system

/tablegen
- Windows command line program (mingw)
- generates tables for cube solver algorithm


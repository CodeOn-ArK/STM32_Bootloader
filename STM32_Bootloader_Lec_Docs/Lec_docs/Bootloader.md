
#                            Bootloader 
#_____________________________________________________________________

Bootloader is a software stored in the MCU flash ot ROM to act as an application loader as well as 
a mechanism to update the applications whenever required.
Bootloaders are provided in nearly all uCs. In absence of an external circuitry like ICDP(In-Circuit 
Debugger/Programmer), the bootloader acts as a piece of software which programs the flash of the chip.
Bootloaders are used in IAP(In-Application Programming).

##Arduino example: 

The atmega328p uC also has a proprietary bootloader in the flash/ROM. Whenever the system undergoes RESET
the Bootloader is the first program to run in the uC. Bootloader then gives the control to the Application
stored in FLASH in absence of signal from host.
During programming the chip, the host RESETs the uC first than signals the Bootloader to start. The Bootloader 
then receives the code from host and programs it into FLASH. After completion the host RESETs the board again
and then the application starts working.

##        Mmory Organization - STM32F446RE

Embedded Flash          = 512KB
Internal SRAM           = 112KB
Internal SRAM2          = 16KB
System Memory(ROM)      = 30KB -- Bootloader is here
OTP Memory              = 528B 
Option Bytes            = 16B
Backup RAM              = 4KB

- [x] Internal FLASH Memory

 - Size         512KB
 - Begins @     0x0080 0000
 - Ends @       0807 FFFF
 - Usage :      Used to store application code an RO data of the Prog && vector table
 - Volatility : Non - volatile

- [x] Internal SRAM1

 - Size         112KB
 - Begins @     0x2000 0000
 - Ends @       0x2001 BFFF
 - Usage :      Used to application global data, static variable
.               Also Used for Stack and Heap Purpose
 - Volatility : Volatile

- [x] Internal SRAM2

 - Size         16KB
 - Begins @     0x2001 C000
 - Ends @       0x2001 FFFF
 - Usage :      Used to application global data, static variable
.                Also Used for Stack and Heap Purpose
 - Volatility : Volatile

- [x] ROM

 - Size         30KB
 - Begins @     0x1FFF 0000
 - Ends @       0x1FFF 77FF
 - Usage :      By default uC will not execute any code from this memory,
.               but can be configured to boot or exdcute bootloader from this mem
 - Volatility : RO mem


##               RESET sequence in STM32

1) Upon RESET, the PC of the processor is loaded with the value 0x0000 0000

2) Then processor loads the value@mem location 0x0000 0000 into the MSP

MSP = value@0x0000 0000 ; processor first initializes the Stack Pointer register

3) After this processor reads the value@mem location 0x0000 0004 into PC
That value is actually address of the Reset Handler.

4) PC jumps to RESET handler

5) Actually the FLASH area starts from 0x0800 0000 but the contents are fetched from 0x0000 0000
   this is achieved by memory aliasing. The processor produces the address 0x0000 0000 which is aliased
   internally to 0x0800 0000. From 0x0800 0004 ( 0x0000 0004 ) the vector table starts.


##                  Boot Configuration

   |-------------------------------------------------------------------------------------
   |Boot Mode Selection pins|                   |
   |------------------------|                   |
   | BOOT1     |   BOOT2    |     Boot Mode     |               Aliasing
   |-------------------------------------------------------------------------------------
   |           |            |                   |
   |     X     |     0      |   Main Flash Mem  |  Main FLASH memory is selected as the boot Area
   |           |            |                   |
   |     0     |     1      |   System Memory   |  System memory is selected as the boot area
   |           |            |                   |
   |     1     |     1      |   Embedded SRAM   |  Embedded SRAM is selcted as the boot area
   |_________________|__________________|_________________________|______________________________________________

![plot](/home/ark/st/STM32_Bootloader/STM32_Bootloader_Lec_Docs/Lec_docs/Screenshot_3.jpg)


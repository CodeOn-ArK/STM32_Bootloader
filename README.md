# STM32_CustomBootloader

This Repo exploares the various possibility and basuc implementation of STM32 Bootloader  

![click here](https://github.com/CodeOn-ArK/STM32_CustomBootloader/blob/master/STM32_Bootloader_Lec_Docs/Lec_docs/Bootloader.md) to know more about bootloaders and their working

## How to use this?

The bootloader is programmed in the first 2 sectors of the stm32. Leaving the rest for User application.
You can communicate with the bootloader by following these steps :-

- Press and hold down the User button(PC13) while the board is RESETed
- Now you have entered the bootloader mode. Use the python program to communicate
  with the bootloader.
- If the first condition is not followed than the uC starts running the USER application
  starting from sector 2

  additions - I will add a QT application for this in near future

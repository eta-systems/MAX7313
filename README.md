# MAX7313 C and C++ Libraries
### 16-Port I/O Expander with LED Intensity Control 

This library is made for the **STM32 HAL** (Hardware Abstraction Library) platform. See notes below to adjust the code for other platforms. The example code is for STM32CubeMX and Keil uVision 5 IDE.

---

### Usage

```c
/* USER CODE BEGIN Includes */
#include "max7313.h"          // include the library
```

```c
/* USER CODE BEGIN 0 */
#define MAX7313_address 0x42        // 100 0010
MAX7313 ioDriver_1;
/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 2 */

    ioDriver_1 = new_MAX7313(); // initialize the default values
    MAX7313_Pin_Mode(&ioDriver_1, 1, PORT_OUTPUT);
    MAX7313_Pin_Mode(&ioDriver_1, 2, PORT_INPUT);
    MAX7313_Init(&ioDriver_1, &hi2c1, MAX7313_address); // initialize the port expander chip
    MAX7313_Interrupt_Enable(&ioDriver_2); // must be called after Init()
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        MAX7313_Pin_Write(&ioDriver_1, 1, 15); // 15 = max
        HAL_Delay(500);
        MAX7313_Pin_Write(&ioDriver_1, 1, 0);  //  0 = min
        HAL_Delay(500);
    }
  /* USER CODE END 3 */

}
```

---
### Examples

The examples provided were tested on custom hardware (STM32F373). It displays the usage of the interrupt functionality to blink an LED in the main loop while reacting to button presses in the interrupt handler.

---

### Restrictions

This library only includes functionality to use the pins on the MAX7313 port expander as inputs or PWM outputs. The Blink functionality is not implemented.

You should be able to port this library to any other platform which supports C or C++. Note the comments inside the code. There are the hardware glue functions (methods) `MAX7313_Read8` (`MAX7313::read8`) and `MAX7313_Write8` (`MAX7313::write8`) which you will have to change to your I2C interface.

I recommend using the C library instead of the C++ library since it is more lightweight on microcontrollers.

---

### Sources

MAX7313 Datasheet: [maximintegrated.com](https://datasheets.maximintegrated.com/en/ds/MAX7313.pdf)

Arduino Code: [forum.arduino.cc](https://forum.arduino.cc/index.php?topic=9682.0)

Enable C++ support in Keil uVision: [github.com/iancanada](https://github.com/iancanada/STM32Cube-Cpp-programming-example)

Arduino library for MAX7313: [github.com/weirdchaos](https://github.com/weirdchaos/MAX7313)

---

### License

MIT License

Copyright (c) 2018 Simon Burkhardt (github.com/mnemocron)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.



# MAX7313 C++ Library
### 16-Port I/O Expander with LED Intensity Control 

This library is made for the **STM32 HAL** (Hardware Abstraction Library) platform. See notes below to adjust the code for other platforms. The example code is for STM32CubeMX and Keil uVision 5 IDE.

---

### Usage

```cpp
/* USER CODE BEGIN Includes */
#include "max7313.h"          // include the library
```

```cpp
/* USER CODE BEGIN 0 */
#define MAX7313_1 0x42        // 100 0010

MAX7313 ioDriver(&hi2c1, MAX7313_1);   // declare new port expander chip on hi2c interface

MAX7313Output LED_0    (&ioDriver_2, 8, 0);  // new output on port 8, active HIGH
MAX7313Input  Button_0 (&ioDriver_2, 2);     // new input on port 8
/* USER CODE END 0 */

int main(void)
{

    /* USER CODE BEGIN 2 */
    ioDriver.begin();         // initialize the port expander chip
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        LED_0.setIntensity(15);     // 15 = max brightness
        HAL_Delay(500);
        LED_0.setIntensity(0);      //  0 = off
        HAL_Delay(500);
    }
  /* USER CODE END 3 */

}
```

### Interrupts

enable EXTI/GPIO interrupt with **falling edge** detection in the configuration and handle the respective interrupts in `stm32f3xx_it.c`

```cpp

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  // handle data change interrupt
  if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_5)){ 
        // handle the interrupt, eg. read out the button value
        // when an interrupt occurs, the interrupt must be reset on the port expander
        // use .clearInterrupt() or just read out the input registers
  }

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}
```

---
### Example

The example provided was tested on custom hardware. It displays the usage of the interrupt functionality to blink an LED in the main loop while reacting to button presses in the interrupt handler.

---

### Restrictions

This library only includes functionality to use the pins on the MAX7313 port expander as inputs or PWM outputs. The Blink functionality is not implemented.

You should be able to port this library to any other platform which supports C++. Note the comments inside the code. There are the hardware glue functions `MAX7313::read8` and `MAX7313::write8` which you will have to change to your I2C interface.

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



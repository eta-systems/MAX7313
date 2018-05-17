# C++ Example Code

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


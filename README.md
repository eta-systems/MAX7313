# MAX7313 C++ Library
### 16-Port I/O Expander with LED Intensity Control 

This library is currently in development for the STM32 HAL platform.
Development platforms include STM32CubeMX and Keil uVision 5 IDE.

### Sources

Datasheet: [maximintegrated.com](https://datasheets.maximintegrated.com/en/ds/MAX7313.pdf)

Arduino Code: [forum.arduino.cc](https://forum.arduino.cc/index.php?topic=9682.0)

Enable C++ support in Keil uVision: [github.com/iancanada](https://github.com/iancanada/STM32Cube-Cpp-programming-example)

### Example Code

```cpp
/* USER CODE BEGIN Includes */
#include "max7313.h"

.
.
.

/* USER CODE BEGIN 0 */
#define MAX7313_1 0x42        // 100 0010
#define MAX7313_2 0x44        // 100 0100

MAX7313 ioDriver_1(&hi2c1, MAX7313_1);
MAX7313 ioDriver_2(&hi2c1, MAX7313_2);

MAX7313Output LED_0 (&ioDriver_2, 8, 0);
MAX7313Output LED_1 (&ioDriver_2, 9, 0);
MAX7313Output LED_2 (&ioDriver_2,10, 0);
MAX7313Output LED_3 (&ioDriver_2,14, 0);
MAX7313Output LED_4 (&ioDriver_2,15, 0);
MAX7313Output LED_5 (&ioDriver_1, 8, 0);
MAX7313Output LED_6 (&ioDriver_1, 9, 0);
MAX7313Output LED_7 (&ioDriver_1,13, 0);
MAX7313Output LED_8 (&ioDriver_1,14, 0);
MAX7313Output LED_9 (&ioDriver_1, 4, 0);
MAX7313Output LED_10(&ioDriver_1, 1, 0);
MAX7313Input  Button_SW4 (&ioDriver_2, 1);
MAX7313Input  Button_SW3 (&ioDriver_2, 2);

.
.
.

/* USER CODE END 0 */

int main(void)
{

    .
    .
    .

    /* USER CODE BEGIN 2 */
    ioDriver_1.begin();
    ioDriver_2.enableInterrupt();
    ioDriver_2.begin();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        LED_0.setIntensity(15);
        HAL_Delay(500);
        LED_0.setIntensity(0);
        HAL_Delay(500);
    }
  /* USER CODE END 3 */

}
```

### Interrupts

enable EXTI interrupts in the configuration and handle the respective interrupts in `stm32f3xx_it.c`

```cpp
/* USER CODE BEGIN 0 */
extern uint8_t interrupt_val;
/* USER CODE END 0 */

.
.
.

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  // handle data change interrupt
  if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_5)){
        interrupt_val = 15 - interrupt_val;
  }

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}


```



snippet from main.c code

```cpp
uint8_t interrupt_val = 0;

// Cled k(3);   // Test if C++ libraries actually work

/* USER CODE END 0 */

int main(void)
{

  .
  .
  .

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
        
        LED0.setIntensity(interrupt_val);
        
        if(Button_SW3.read())
            LED1.setIntensity(15);
        else
            LED1.setIntensity(0);
        
        if(Button_SW4.read())
            LED2.setIntensity(15);
        else
            LED2.setIntensity(0);
        
        HAL_Delay(100);
        
  }
  /* USER CODE END 3 */

  .
  .
  .

}
```

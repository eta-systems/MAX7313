# MAX7313 C++ Library (for STM32 HAL)
### 16-Port I/O Expander with LED Intensity Control 

## This library is still in development.

### Sources

Datasheet: [maximintegrated.com](https://datasheets.maximintegrated.com/en/ds/MAX7313.pdf)

Arduino Code: [forum.arduino.cc](https://forum.arduino.cc/index.php?topic=9682.0)

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




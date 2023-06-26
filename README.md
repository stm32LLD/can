# **STM32 CAN Low Level Driver**
Following repository constains STM32 CAN Low Level (LL) driver C implementation based on STM32 HAL library.

Module shall support all STM32 device famility, as all STM32 MCU incorporates CAN periphery.

## **Dependencies**

### **1. STM32 HAL library**
STM32 CAN LL driver module uses STM32 HAL library.


## **API**
| API Functions | Description | Prototype |
| --- | ----------- | ----- |
| **can_init** | Initialization of CAN module | can_status_t can_init(void) |


## **Usage**

**GENERAL NOTICE: Put all user code between sections: USER CODE BEGIN & USER CODE END!**

**1. Copy template files to root directory of the module**

Copy configuration file *can_cfg* to root directory and replace file extension (.htmp/.ctmp -> .h/.c).

**2. Change default HAL library include to target microprocessor inside ***can_cfg.h***:**

Following example shows HAL library include for STM32L4 family:
```C
// USER INCLUDE BEGIN...

#include "stm32l4xx_hal.h"

// USER INCLUDE END...
```

**3. Configure CAN module for application needs by changing ***can_cfg.h***. Configuration options are following:**

| Configuration | Description |
| --- | --- |
| **CAN_CFG_ASSERT_EN** 		        | Enable/Disable assertions |
| **CAN_ASSERT** 		                | Assert definition |


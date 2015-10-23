#include <stm32f4xx_gpio.h>
#include "stm32f4xx.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "stm32f4xx_gpio.h"

#include "diagnostics.h"

static void init_leds (void);

int main(void) {

    SystemInit();

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    diag_init();

    vTaskStartScheduler();
}


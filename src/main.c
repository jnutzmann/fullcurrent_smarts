#include "stm32f4xx.h"
#include "misc.h"


int main(void) {

  SystemInit();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  for (;;);

}


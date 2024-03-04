###### DESCRIPTION

`The given code is to setup Gpio interrupt `

## Getting started

Firstly, we will start by including the necessary libraries for this project. This includes the driver/gpio.h library as we have to work with GPIO pins and FreeRTOS libraries as we want to create tasks and queues.

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
```

We are using boot button on Esp32 Dev kit 1 which is connected to GPIO0. Therefore, we will define a variable called ‘CONFIG_INPUT_PIN’ that will hold the GPIO pin 0. This will be used to read the input as interrupt. To use other Gpio change CONFIG_INPUT_PIN number.

The onboard LED of ESP32 is connected with GPIO2. Therefore, we will define a variable called ‘LED_PIN’ that will hold the GPIO pin 2. This will be used later on in the code for the indication of Wifi_AP mode

```c
#define CONFIG_INPUT_PIN 0
#define LED_PIN 2
```

The gpio_interrupt_handler is the interrupt handler or ISR. It is called whenever an interrupt occurs.

Inside the interrupt service routine we will call the xQueueSendFromISR() function. It will send an item from a queue. This function takes in three parameters. The first parameter is ‘xQueue’ which is the queue handle on which the item is to be posted. In our case it is ‘interuptQueue.’ The second parameter is the ‘pvItemToQueue’ which is the pointer to the item that is placed on the queue. In our case it is ‘&pinNumber.’ The third parameter is the ‘pxHigherPriorityTaskWoken’ which is NULL in our case.

```c
static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}
```

The Interrupt_Task function is given below. Inside this function, we first create two integer variables ‘pinNumber’ and ‘count’ and set their value to 0. Then inside the infinite while function, we check if an item is received from the queue by using the xQueueReceive() function. This function takes in three parameters. The first parameter is the ‘xQueue’ which is the queue handle from which the item is to be received. It is ‘interuptQueue’ in our case. The second parameter is the ‘pvBuffer’ which is the pointer to the buffer into which the received item will be copied. In our case it is ‘&pinNumber.’ The third parameter is the ‘xTicksToWait’ which is the maximum amount of time the task should block waiting for an item to receive should the queue be empty at the time of the call. In our case it is set as ‘portMAX_DELAY.’ Thus, when the interrupt occurs then Interrupt_Task is called

```c
void Interrupt_Task(void *params)
{
  bool state = false;
  int pinNumber, count = 0;
    while (true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            state = !state;
            gpio_set_level(LED_PIN, state);
            printf("button pressed %d times\n", ++count);
            //Write Code here
   

        }
    }
}


void Init_Interrupt(void)
{
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(CONFIG_INPUT_PIN);
    gpio_set_direction(CONFIG_INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(CONFIG_INPUT_PIN);
    gpio_pullup_dis(CONFIG_INPUT_PIN);
    gpio_set_intr_type(CONFIG_INPUT_PIN, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_INPUT_PIN, gpio_interrupt_handler, (void *)CONFIG_INPUT_PIN);
    interputQueue = xQueueCreate(10, sizeof(int));
}

//------------------------------
//MAIN

  xTaskCreate(Interrupt_Task, "Interrupt_Task", 2048, NULL, 1, NULL);
```

Here Init_Interrupt function initlize the interrupt for CONFIG_INPUT_PIN along with the initilization of gpio 2 for LED.

Lastly xTaskCreate function is called in main along.

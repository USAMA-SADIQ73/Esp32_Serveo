#include <stdio.h>
#include <stdio.h>
#include <Interrupt_Gpio.h>
#include "driver/gpio.h"
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "servo_sg90.h"
#include "esp32_i2c.h"
#include "esp_log.h"


#define Interrupt_Button 0
#define Adc_PIN 35
#define Servo_PIN   26

void app_main(void)
{
    


    Init_Interrupt_Adc(Interrupt_Button, Adc_PIN);
    Init_Servo_SG90(Servo_PIN);
    xTaskCreate(Interrupt_Task, "Interrupt_Task", 2048, NULL, 1, NULL);
    xTaskCreate(i2c_task, "i2c_task", 2048, (void*)0, 1, NULL);
    xTaskCreate(servo_task, "servo_task", 2048, NULL, 1, NULL);
    
    // while(1)
    // {
        
        
    //     //printf("Count = %d\n",get_count());
    //     //printf("Adc Value = %f\n",ADC_value());
    //     //printf("Adc Raw Value = %ld\n",ADC_Raw_value());
    //     //printf("Slave Data main = %ld\n", get_Slave_data());
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

}

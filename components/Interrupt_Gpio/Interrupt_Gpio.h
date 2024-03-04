
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"



void IRAM_ATTR gpio_interrupt_handler(void *args);
void Interrupt_Task(void *params);
esp_err_t Init_Interrupt_Adc(int Int_Gpio,int Adc_Gpio);
int get_count(void);
void set_count(int value);
uint32_t ADC_Raw_value(void);
float ADC_value(void);
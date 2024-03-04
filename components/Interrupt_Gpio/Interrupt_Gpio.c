
#include "Interrupt_Gpio.h"
#include "driver/adc.h"
#include "esp_random.h"



QueueHandle_t interputQueue;
volatile int pulseCount;  

uint32_t ADC_Raw = 0;
float ADC35_value = 0;

void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);  
}

void Interrupt_Task(void *params)
{
    int8_t pinNumber = 0;
    while (1)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
        pulseCount++;
        //Take Adc value  
        ADC_Raw = adc1_get_raw(ADC1_CHANNEL_7);
        printf("intterrupt = %ld\n", ADC_Raw);
       // vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

}
int get_count(void)
{
    return pulseCount;
}
void set_count(int value)
{
    pulseCount = value;
}

float ADC_value(void)
{
    return ADC35_value;
    //return esp_random()/10000;
}

uint32_t ADC_Raw_value(void)
{
    return ADC_Raw;
}

esp_err_t Init_Interrupt_Adc(int Int_Gpio,int Adc_Gpio)
{
    
    esp_rom_gpio_pad_select_gpio(Int_Gpio);
    gpio_set_direction(Int_Gpio, GPIO_MODE_INPUT);
    gpio_pulldown_en(Int_Gpio);
    gpio_pullup_dis(Int_Gpio);
    gpio_set_intr_type(Int_Gpio, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(Int_Gpio, gpio_interrupt_handler, (void *)Int_Gpio);
    interputQueue = xQueueCreate(10, sizeof(int));

    if (Adc_Gpio != GPIO_NUM_35) 
    {
        return ESP_ERR_INVALID_ARG;
    }
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    return ESP_OK;
}









typedef struct {
    int Arg;
    int Adc_master_write_slave;
    int Adc_slave_read_master;
} i2c_task_params_t;

void disp_buf(uint8_t *buf, int len);
void i2c_task(void *arg);
uint32_t get_Slave_data(void);
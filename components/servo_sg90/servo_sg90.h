

typedef struct {
    int angle;
    int step;
} servo_task_params_t;

uint32_t example_angle_to_compare(int angle);
void Init_Servo_SG90(int Servo_Gpi0);
void servo_task(void *pvParameters);
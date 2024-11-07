#include "main.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define ADC_THRESHOLD 2000  // Threshold for object detection
#define TASK_STACK_SIZE 128
#define MAX_PRIORITY 5      // Highest priority level
#define MED_PRIORITY 3
#define MIN_PRIORITY 1

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
SemaphoreHandle_t xMutex;

typedef struct {
    TaskHandle_t handle;
    uint32_t period;         // Task period (ms)
    uint32_t deadline;       // Task absolute deadline (ms)
    uint32_t lastWakeTime;   // Task's last wake time
} TaskParams;

TaskParams IR_Task_Params, LED_Task_Params;

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void IR_Task_Handler(void* parameters);
static void LED_Control_Task_Handler(void* parameters);
void schedule_EDF();
void schedule_LLF();
void PCP_Enter(SemaphoreHandle_t mutex, TaskHandle_t task);
void PCP_Exit(SemaphoreHandle_t mutex);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    xMutex = xSemaphoreCreateMutex();
    configASSERT(xMutex != NULL);

    // Initialize task parameters
    IR_Task_Params.period = 100;
    IR_Task_Params.deadline = 100;
    IR_Task_Params.lastWakeTime = xTaskGetTickCount();

    LED_Task_Params.period = 1000;
    LED_Task_Params.deadline = 1000;
    LED_Task_Params.lastWakeTime = xTaskGetTickCount();

    // Create tasks
    xTaskCreate(IR_Task_Handler, "IR Sensor Task", TASK_STACK_SIZE, &IR_Task_Params, MED_PRIORITY, &IR_Task_Params.handle);
    xTaskCreate(LED_Control_Task_Handler, "LED Control Task", TASK_STACK_SIZE, &LED_Task_Params, MIN_PRIORITY, &LED_Task_Params.handle);

    vTaskStartScheduler();

    while (1) { /* Never reached */ }
}

/* System Clock Configuration */
void SystemClock_Config(void) {
    // Clock configuration code here
}

/* Initialize peripherals */
static void MX_GPIO_Init(void) { /* GPIO initialization */ }
static void MX_ADC1_Init(void) { /* ADC initialization */ }
static void MX_USART2_UART_Init(void) { /* UART initialization */ }

/* PCP functions to enter and exit mutex with ceiling priority */
void PCP_Enter(SemaphoreHandle_t mutex, TaskHandle_t task) {
    // Set the task’s priority to ceiling priority before entering critical section
    vTaskPrioritySet(task, MAX_PRIORITY);
    xSemaphoreTake(mutex, portMAX_DELAY);
}

void PCP_Exit(SemaphoreHandle_t mutex) {
    xSemaphoreGive(mutex);
    // Restore task's priority (assuming MED_PRIORITY for simplicity)
    vTaskPrioritySet(xTaskGetCurrentTaskHandle(), MED_PRIORITY);
}

/* EDF scheduling */
void schedule_EDF() {
    uint32_t IR_Deadline = IR_Task_Params.lastWakeTime + IR_Task_Params.deadline;
    uint32_t LED_Deadline = LED_Task_Params.lastWakeTime + LED_Task_Params.deadline;

    if (IR_Deadline <= LED_Deadline) {
        vTaskPrioritySet(IR_Task_Params.handle, MAX_PRIORITY);
        vTaskPrioritySet(LED_Task_Params.handle, MIN_PRIORITY);
    } else {
        vTaskPrioritySet(LED_Task_Params.handle, MAX_PRIORITY);
        vTaskPrioritySet(IR_Task_Params.handle, MIN_PRIORITY);
    }
}

/* LLF scheduling */
void schedule_LLF() {
    uint32_t IR_Laxity = IR_Task_Params.deadline - (xTaskGetTickCount() - IR_Task_Params.lastWakeTime);
    uint32_t LED_Laxity = LED_Task_Params.deadline - (xTaskGetTickCount() - LED_Task_Params.lastWakeTime);

    if (IR_Laxity <= LED_Laxity) {
        vTaskPrioritySet(IR_Task_Params.handle, MAX_PRIORITY);
        vTaskPrioritySet(LED_Task_Params.handle, MIN_PRIORITY);
    } else {
        vTaskPrioritySet(LED_Task_Params.handle, MAX_PRIORITY);
        vTaskPrioritySet(IR_Task_Params.handle, MIN_PRIORITY);
    }
}

/* Task to handle IR sensor */
static void IR_Task_Handler(void* parameters) {
    TaskParams* taskParams = (TaskParams*)parameters;
    uint32_t adc_value;
    char message[50];

    while (1) {
        schedule_EDF();  // Select EDF or switch to schedule_LLF() for LLF
        PCP_Enter(xMutex, taskParams->handle);

        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
            adc_value = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);

        snprintf(message, sizeof(message), "ADC Value: %lu\r\n", adc_value);
        HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

        PCP_Exit(xMutex);

        if (adc_value > ADC_THRESHOLD) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        }

        taskParams->lastWakeTime = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(taskParams->period));
    }
}

/* LED Control Task Handler */
static void LED_Control_Task_Handler(void* parameters) {
    TaskParams* taskParams = (TaskParams*)parameters;

    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        taskParams->lastWakeTime = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(taskParams->period));
    }
}

/* Error Handler */
void Error_Handler(void) {
    __disable_irq();
    while (1) { }
}

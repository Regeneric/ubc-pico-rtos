#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"



// VSS and INJ
static float INJECTION_VALUE = 0.0025f;
static float PULSE_DISTANCE  = 0.00006823;

typedef struct {
    volatile uint distPulseCount;
    volatile uint speed;
    volatile uint avgSpeedCount;

    float traveledDistance;
    volatile float sumInv;
    volatile float avgSpeedDivider;
} vssData; vssData vss;

typedef struct {
    volatile uint32_t timeHigh;
    volatile uint32_t timeLow;
    volatile uint32_t pulseTime;

    volatile float fuelSumInv;
    volatile float openTime;
    volatile float insFuelConsumption;
    volatile float avgFuelConsumption;
} injData; injData inj;

#ifndef INJ_PIN
#define INJ_PIN 21
#endif

#ifndef VSS_PIN
#define VSS_PIN 22
#endif

#ifndef NO_INJ
#define NO_INJ 4
#endif


void initIRQ(uint vssGPIO, uint injGPIO);
void ISR();


void vssTask(void *params);
TaskHandle_t vssTaskHandle = NULL;
static QueueHandle_t vssDataQu = NULL;

void injTask(void *params);
TaskHandle_t injTaskHandle = NULL;
static QueueHandle_t injDataQu = NULL;


void currentSpeedTask(void *params);
TaskHandle_t currentSpeedTaskHandle = NULL;
void avgSpeedTask(void *params);
TaskHandle_t avgSpeedTaskHandle = NULL;


void fuelConsumptionTask(void *params);
TaskHandle_t fuelConsumptionTaskHandle = NULL;


void systemTask(void *params);
TaskHandle_t systemTaskHandle = NULL;
// _VSS and INJ_


void displayTask(void *params);
TaskHandle_t displayTaskHandle = NULL;


void initTimers();
void displayTimerISR();
void sysTimerISR();


inline void initScreen() {printf("\033[2J");}
void displayTimerTask(void *params);
TaskHandle_t displayTimerTaskHandle = NULL; 
TimerHandle_t displayTimer = NULL;

void sysTimerTask(void *params);
TaskHandle_t sysTimerTaskHandle = NULL;
TimerHandle_t sysTimer = NULL;


int main() {
    stdio_init_all();
    initIRQ(VSS_PIN, INJ_PIN);
    

    // TimerHandle_t timers[2] = {displayTimer, sysTimer};
    initTimers();
    if(xTimerStart(displayTimer, 0) != pdPASS) while(1);
    if(xTimerStart(sysTimer, 0) != pdPASS) while(1);

    initScreen();
    xTaskCreate(displayTask, "Display Task",
        256, NULL,
        1, &displayTaskHandle
    );


    // VSS and INJ
    vssDataQu = xQueueCreate(1, sizeof(vssData));
    injDataQu = xQueueCreate(1, sizeof(injData));

        vss.speed = 0;
        vss.avgSpeedCount = 0;
        vss.distPulseCount = 0;

        vss.sumInv = 0.0;
        vss.avgSpeedDivider = 0.0;
        vss.traveledDistance = 0.0;        
        

        inj.timeHigh  = 0;
        inj.timeLow   = 0;
        inj.pulseTime = 0;

        inj.openTime = 0.0;
        inj.fuelSumInv = 0.0;
        inj.insFuelConsumption = 0.0;
        inj.avgFuelConsumption = 0.0;

    xTaskCreate(vssTask, "VSS ISR",
        256, NULL,
        2, &vssTaskHandle
    );
    xTaskCreate(injTask, "INJ ISR",
        256, NULL,
        2, &injTaskHandle
    );

    xTaskCreate(currentSpeedTask, "Current vehicle speed",
        256, NULL,
        1, &currentSpeedTaskHandle
    );
    xTaskCreate(avgSpeedTask, "Average vehicle speed",
        256, NULL,
        2, &avgSpeedTaskHandle
    );

    xTaskCreate(fuelConsumptionTask, "Fuel consumption",
        256, NULL,
        1, &fuelConsumptionTaskHandle
    );

    xTaskCreate(systemTask, "VSS and INJ instan values",
        256, NULL,
        1, &systemTaskHandle
    );
    // _VSS and INJ_


    vTaskStartScheduler();
    while(1) {}

    return 0;
}


// VSS and INJ
void initIRQ(uint vssGPIO, uint injGPIO) {
    gpio_init(vssGPIO); 
    gpio_init(injGPIO);
    
    gpio_set_dir(vssGPIO, GPIO_IN); 
    gpio_set_dir(injGPIO, GPIO_IN);
    
    gpio_pull_up(vssGPIO); 
    gpio_pull_up(injGPIO);

    // Both IRQs use the same ISR
    gpio_set_irq_enabled_with_callback(vssGPIO, GPIO_IRQ_EDGE_FALL, 1, ISR);
    gpio_set_irq_enabled(injGPIO, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 1);
}
void ISR(uint gpio) {
    BaseType_t checkYield;

    // Logic inside ISR decides, which task should be resumed
    if(gpio == VSS_PIN) checkYield = xTaskResumeFromISR(vssTaskHandle);
    if(gpio == INJ_PIN) checkYield = xTaskResumeFromISR(injTaskHandle);

    portYIELD_FROM_ISR(checkYield);
}


void vssTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);     // NULL suspends itself
        
        vss.distPulseCount += 1;
        vss.traveledDistance += PULSE_DISTANCE;

        xQueueSend(vssDataQu, &vss, 0U);
        // vTaskResume(currentSpeedTaskHandle);
    } return;
}

void injTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);     // NULL suspends itself
        
        if(gpio_get(INJ_PIN) == 0) inj.timeLow = xTaskGetTickCount();   // Low state on INJ_PIN
        else {
            // High state on INJ_PIN
            inj.timeHigh = xTaskGetTickCount();
            inj.pulseTime = inj.pulseTime + ((inj.timeHigh/portTICK_PERIOD_MS) - (inj.timeLow/portTICK_PERIOD_MS));
            gpio_put(INJ_PIN, 0);

            xQueueSend(injDataQu, &inj, 0U);
            // vTaskResume(fuelConsumptionTaskHandle);
        }
    } return;
}


void currentSpeedTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);

        if(uxQueueMessagesWaiting(vssDataQu) > 0) {
            xQueueReceive(vssDataQu, &vss, portMAX_DELAY);
            
            vss.speed = PULSE_DISTANCE * vss.distPulseCount * 3600;
            if(vss.speed > 5) vss.avgSpeedDivider += 1.0f;

            xQueueSend(vssDataQu, &vss, 0U);
            vTaskResume(avgSpeedTaskHandle);
        }
    } return;
}

void avgSpeedTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);

        if(uxQueueMessagesWaiting(vssDataQu) > 0 && vss.speed > 5) {
            xQueueReceive(vssDataQu, &vss, portMAX_DELAY);

            // Harmonic mean
            vss.sumInv = vss.sumInv + (1.0f/(float)vss.speed);
            vss.avgSpeedCount = vss.avgSpeedDivider/vss.sumInv;

            xQueueSend(vssDataQu, &vss, 0U);
        }
    } return;
}


void fuelConsumptionTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);

        if(uxQueueMessagesWaiting(injDataQu) > 0) {
            xQueueReceive(injDataQu, &inj, portMAX_DELAY);

            int x = NO_INJ * 3600;  // 3600 (seconds in hour) * no. of injectors

            float iotv = (inj.openTime * INJECTION_VALUE) * x;
            float inv  = (inj.openTime * INJECTION_VALUE) * NO_INJ;
            inj.openTime = ((float)inj.pulseTime/1000);     // Converting to seconds
                    
            if(vss.speed > 5) {
                inj.insFuelConsumption = (100*iotv)/(float)vss.speed;

                // Harmonic mean
                if(inj.insFuelConsumption > 0 && inj.insFuelConsumption < 100) {
                    inj.fuelSumInv = inj.fuelSumInv + (1.0f/inj.insFuelConsumption);
                    inj.avgFuelConsumption = inj.avgFuelConsumption/inj.fuelSumInv;
                }
            } else inj.insFuelConsumption = iotv;

            xQueueSend(injDataQu, &inj, 0U);
        }
    } return;
}

void systemTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);

        if(uxQueueMessagesWaiting(vssDataQu) > 0) {
            xQueueReceive(vssDataQu, &vss, portMAX_DELAY);
            vss.distPulseCount = 0;
            xQueueSend(vssDataQu, &vss, 0U);
        } 

        if(uxQueueMessagesWaiting(injDataQu) > 0) {
            xQueueReceive(injDataQu, &inj, portMAX_DELAY);
            inj.pulseTime = 0;
            xQueueSend(injDataQu, &inj, 0U);
        }
    } return;
}
// _VSS and INJ_


void initTimers() {
    sysTimer = xTimerCreate("SYS TIMER", pdMS_TO_TICKS(1000), pdTRUE, 0, sysTimerISR);
    if(sysTimer == NULL) while(1);

    displayTimer = xTimerCreate("DISPLAY TIMER", pdMS_TO_TICKS(250), pdTRUE, 0, displayTimerISR);
    if(displayTimer == NULL) while(1);
}
void displayTimerISR() {
    BaseType_t checkYield;
    xTaskResumeFromISR(displayTaskHandle);
    portYIELD_FROM_ISR(checkYield);
}
void sysTimerISR() {
    BaseType_t checkYield;

    xTaskResumeFromISR(currentSpeedTaskHandle);
    xTaskResumeFromISR(fuelConsumptionTaskHandle);
    xTaskResumeFromISR(systemTaskHandle);

    portYIELD_FROM_ISR(checkYield);
}


void displayTask(void *params) {
    vssData vd; injData id;

    while(1) {
        vTaskSuspend(NULL);
        // printf("\033[2J");  // Clear screen

        if(uxQueueMessagesWaiting(vssDataQu) > 0 && uxQueueMessagesWaiting(injDataQu) > 0) {
            xQueueReceive(vssDataQu, &vd, portMAX_DELAY);
            xQueueReceive(injDataQu, &id, portMAX_DELAY);

            printf("\033[1;1H");    // Move cursor to (1, 1)
            printf("Cur speed: %d  \r", vd.speed);
        
            printf("\033[3;1H");    // Move cursor to (3,1)
            printf("Ins fuel cons: %.1f  \r", id.insFuelConsumption);


            printf("\033[2;1H");    // Move cursor to (2, 1)
            printf("Avg speed: %d  \r", vd.avgSpeedCount);

            printf("\033[4;1H");    // Move cursor to (4,1)
            printf("Avg fuel cons: %.1f  \r", id.avgFuelConsumption);
        } 
    } return;
}
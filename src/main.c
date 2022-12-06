#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"


// VSS and INJ
#ifndef INJ_PIN
#define INJ_PIN 21
#endif

#ifndef VSS_PIN
#define VSS_PIN 22
#endif

#ifndef NO_INJ
#define NO_INJ 4
#endif

static float g_injectionValue = 0.0025f;
static float g_pulseDistance  = 0.00006823;

typedef struct {
    uint avgSpeed;
    uint currentSpeed;
    uint distPulseCount;

    float sumInv;               // For harmonic mean calculations
    float avgSpeedDivider;
    float traveledDistance;
} VssData; VssData g_vss = {0};

typedef struct {
    uint32_t timeHigh;
    uint32_t timeLow;
    uint32_t pulseTime;         // Difference between injector time in high and low state

    float sumInv;               // For harmonic mean calculations
    float openTime;
    float insConsumption;
    float avgConsumption;
} InjData; InjData g_inj = {0};


static QueueHandle_t g_vssDataQu = NULL;
static SemaphoreHandle_t g_vssDataMutex = NULL;

static QueueHandle_t g_injDataQu = NULL;
static SemaphoreHandle_t g_injDataMutex = NULL;


void initDataStructs();
void initIRQ(uint vssGPIO, uint injGPIO);
void ISR();


void initTimer();
void sysTimerISR();

void sysTimerTask(void *params);
TaskHandle_t g_sysTimerTaskHandle = NULL;
TimerHandle_t g_sysTimer = NULL;


void vssTask(void *params);
static TaskHandle_t g_vssTaskHandle = NULL;

void injTask(void *params);
static TaskHandle_t g_injTaskHandle = NULL;
// _VSS and INJ_


void displayTask(void *params);
TaskHandle_t g_displayTaskHandle = NULL;


int main() {
    stdio_init_all();
    initIRQ(VSS_PIN, INJ_PIN);

    initTimer();
    if(xTimerStart(g_sysTimer, 0) != pdPASS) while(1);

    g_vssDataQu = xQueueCreate(1, sizeof(g_vss));
    g_injDataQu = xQueueCreate(1, sizeof(g_inj));

    g_vssDataMutex = xSemaphoreCreateMutex();
    g_injDataMutex = xSemaphoreCreateMutex();


    xTaskCreate(sysTimerTask, "System Timer",
        256, NULL, 3, &g_sysTimerTaskHandle
    );


    xTaskCreate(vssTask, "Vehicle Speed Sensor readings",
        256, NULL, 2, &g_vssTaskHandle
    );

    xTaskCreate(injTask, "Injector signal readings",
        256, NULL, 2, &g_injTaskHandle
    );
    
    
    xTaskCreate(displayTask, "Display data from sensors",
        256, NULL, 1, &g_displayTaskHandle
    );


    vTaskStartScheduler();
    while(1);

    return 0;
}


// void initDataStructs() {}
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

    return;
}
void ISR(uint gpio) {
    BaseType_t checkYield;

    // Logic inside ISR decides, which task should be resumed
    if(gpio == VSS_PIN) checkYield = xTaskResumeFromISR(g_vssTaskHandle);
    if(gpio == INJ_PIN) checkYield = xTaskResumeFromISR(g_injTaskHandle);

    portYIELD_FROM_ISR(checkYield);
    return;
}


void initTimer() {
    g_sysTimer = xTimerCreate("SYS TIMER", pdMS_TO_TICKS(250), pdTRUE, 0, sysTimerISR);
    if(g_sysTimer == NULL) while(1);

    return;
}
void sysTimerISR() {
    BaseType_t checkYield;
    checkYield = xTaskResumeFromISR(g_sysTimerTaskHandle);
    portYIELD_FROM_ISR(checkYield);

    return;
}

void sysTimerTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);

        if(uxQueueSpacesAvailable(g_vssDataQu) > 0) xQueueSend(g_vssDataQu, &g_vss, 0U);
        if(uxQueueSpacesAvailable(g_injDataQu) > 0) xQueueSend(g_injDataQu, &g_inj, 0U);
        vTaskResume(g_displayTaskHandle);

        if(xSemaphoreTake(g_vssDataMutex, 0) == pdTRUE) {
            g_vss.distPulseCount = 0;
            xSemaphoreGive(g_vssDataMutex);
        }

        if(xSemaphoreTake(g_injDataMutex, 0) == pdTRUE) {
            g_inj.pulseTime = 0;
            xSemaphoreGive(g_injDataMutex);
        }
    } return;
}


void vssTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);     // NULL suspends itself
        if(xSemaphoreTake(g_vssDataMutex, 0) == pdTRUE) {
            g_vss.distPulseCount++;
            g_vss.traveledDistance += g_pulseDistance;
            
            // Current vehicle speed
            uint cs = g_pulseDistance * g_vss.distPulseCount * 3600;
            g_vss.currentSpeed = cs;
            
            // Average vehicle speed
            if(g_vss.currentSpeed > 5) {
                g_vss.avgSpeedDivider++;
                
                // Harmonic mean
                g_vss.sumInv   = g_vss.sumInv + (1.0f/(float)g_vss.currentSpeed);
                g_vss.avgSpeed = g_vss.avgSpeedDivider/g_vss.sumInv;
            } xSemaphoreGive(g_vssDataMutex);
        }
    } return;
}


void injTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);     // NULL suspends itself
        if(xSemaphoreTake(g_injDataMutex, 0) == pdTRUE) {
            if(gpio_get(INJ_PIN) == 0) g_inj.timeLow = xTaskGetTickCount();   // Low state on INJ_PIN
            else {
                // High state on INJ_PIN
                g_inj.timeHigh  = xTaskGetTickCount();
                g_inj.pulseTime = g_inj.pulseTime + ((g_inj.timeHigh/portTICK_PERIOD_MS) - (g_inj.timeLow/portTICK_PERIOD_MS));
                gpio_put(INJ_PIN, 0);
            }

            int ni = NO_INJ * 3600;  // no. of injectors * 3600 (seconds in hour)

            float iotv = (g_inj.openTime * g_injectionValue) * ni;
            float inv  = (g_inj.openTime * g_injectionValue) * NO_INJ;
            g_inj.openTime = ((float)g_inj.pulseTime/1000); // Converting to seconds
                    
            if(xSemaphoreTake(g_vssDataMutex, 0) == pdTRUE) {
                if(g_vss.currentSpeed > 5) {
                    g_inj.insConsumption = (100*iotv)/(float)g_vss.currentSpeed;

                    // Harmonic mean
                    if(g_inj.insConsumption > 0) {
                        g_inj.sumInv += (1.0f/g_inj.insConsumption);
                        g_inj.avgConsumption = g_vss.avgSpeedDivider/g_inj.sumInv;
                    }
                } else g_inj.insConsumption = iotv;
                xSemaphoreGive(g_vssDataMutex);
            } xSemaphoreGive(g_injDataMutex);
        }        
    } return;
}


void displayTask(void *params) {
    VssData vd; InjData id;

    while(1) {
        vTaskSuspend(NULL);
        if(uxQueueMessagesWaiting(g_vssDataQu) > 0) {
            xQueueReceive(g_vssDataQu, &vd, portMAX_DELAY);

            printf("\033[1;1H");    // Move cursor to (1, 1)
            printf("Cur speed: %d  \r", vd.currentSpeed);
        
            printf("\033[2;1H");    // Move cursor to (2, 1)
            printf("Avg speed: %d  \r", vd.avgSpeed);
        }

        if(uxQueueMessagesWaiting(g_injDataQu) > 0) {
            xQueueReceive(g_injDataQu, &id, portMAX_DELAY);

            printf("\033[3;1H");    // Move cursor to (3,1)
            printf("Ins fuel cons: %.1f  \r", id.insConsumption);

            printf("\033[4;1H");    // Move cursor to (4,1)
            printf("Avg fuel cons: %.1f  \r", id.avgConsumption);
        }
    } return;
}
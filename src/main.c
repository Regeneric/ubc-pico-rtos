#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/flash.h"
#include "hardware/i2c.h"


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

#ifndef SAVE_FLAG_VAL
#define SAVE_FLAG_VAL 213742069
#endif

static float g_injectionValue = 0.0025f;
static float g_pulseDistance  = 0.00006823;

typedef struct {
    uint saveFlag;

    uint avgSpeed;
    uint currentSpeed;
    uint rangeDistance;
    uint distPulseCount;

    float sumInv;               // For harmonic mean calculations
    float avgSpeedDivider;
    float sailingDistance;
    float traveledDistance;
} VssData; VssData g_vss = {0};

typedef struct {
    uint saveFlag;

    uint32_t timeHigh;
    uint32_t timeLow;
    uint32_t pulseTime;         // Difference between injector time in high and low state

    float sumInv;               // For harmonic mean calculations
    float openTime;
    float fuelLeft;
    float fuelUsed;
    float fuelSaved;
    float insConsumption;
    float avgConsumption;
} InjData; InjData g_inj = {0};


static QueueHandle_t g_vssDataQu = NULL;
static SemaphoreHandle_t g_vssDataMutex = NULL;

static QueueHandle_t g_injDataQu = NULL;
static SemaphoreHandle_t g_injDataMutex = NULL;


void initDataStructs();
void initIRQ(uint vssGPIO, uint injGPIO);
void _ISR();


#ifndef SYS_TIMER_PERIOD_MS
#define SYS_TIMER_PERIOD_MS 1000
#endif

void initTimer();
void _sysTimerISR();

void _sysTimerTask(void *params);
TaskHandle_t g_sysTimerTaskHandle = NULL;
TimerHandle_t g_sysTimer = NULL;


void _vssTask(void *params);
static TaskHandle_t g_vssTaskHandle = NULL;

void _injTask(void *params);
static TaskHandle_t g_injTaskHandle = NULL;
// _VSS and INJ_


void _displayTask(void *params);
TaskHandle_t g_displayTaskHandle = NULL;


#define SAVE_INTERVAL     15
#define FLASH_VSS_OFFSET (256 * 1024)    // 256Kb from the start of flash
#define FLASH_INJ_OFFSET ((256 + sizeof(g_vss)) * 1024)

void initI2C();
inline int reservedAddress(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void _i2cBusScanTask(void *params);
TaskHandle_t g_i2cBusScanTaskHandle = NULL;

void _saveToEEPROMTask(void *params);
TaskHandle_t g_saveToEEPROMTaskHandle = NULL;

void _readFromEEPROMTask(void *params);
TaskHandle_t g_readFromEEPROMTaskHandle = NULL;


void _saveToFlashTask(void *params);
TaskHandle_t g_saveToFlashTaskHandle;

void _readFromFlashTask(void *params);
TaskHandle_t g_readFromFlashTaskHandle;


uint g_coldStart = 1;
uint g_time = 0;


int main() {
    stdio_init_all();
    initIRQ(VSS_PIN, INJ_PIN);

    initI2C();

    initTimer();
    if(xTimerStart(g_sysTimer, 0) != pdPASS) while(1);

    // DEBUG
    g_inj.fuelLeft = 100.0;
    // _DEBUG_

    g_vssDataQu = xQueueCreate(1, sizeof(g_vss));
    g_injDataQu = xQueueCreate(1, sizeof(g_inj));

    g_vssDataMutex = xSemaphoreCreateMutex();
    g_injDataMutex = xSemaphoreCreateMutex();


    xTaskCreate(_saveToEEPROMTask, "Save data to EEPROM mem",
        256, NULL, 5, &g_saveToEEPROMTaskHandle
    );

    xTaskCreate(_readFromEEPROMTask, "Read data from EEPROM mem",
        256, NULL, 5, &g_readFromEEPROMTaskHandle
    );

    // xTaskCreate(_saveToFlashTask, "Saves data to flash mem",
    //     256, NULL, 5, &g_saveToFlashTaskHandle
    // );

    // xTaskCreate(_readFromFlashTask, "Read data from flash mem",
    //     256, NULL, 5, &g_readFromFlashTaskHandle
    // );


    xTaskCreate(_sysTimerTask, "System Timer",
        256, NULL, 3, &g_sysTimerTaskHandle
    );


    // xTaskCreate(_vssTask, "Vehicle Speed Sensor readings",
    //     256, NULL, 2, &g_vssTaskHandle
    // );

    // xTaskCreate(_injTask, "Injector signal readings",
    //     256, NULL, 2, &g_injTaskHandle
    // );
    
    
    // xTaskCreate(_displayTask, "Display data from sensors",
    //     256, NULL, 1, &g_displayTaskHandle
    // );


    vTaskStartScheduler();
    while(1);

    return 0;
}


void initI2C() {
    i2c_init(i2c_default, 100 * 1000);  // 100 kHZ
    
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    return;
}

void _i2cBusScanTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);

        printf("\nI2C Bus Scanner\r\n");
        printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

        for(int addr = 0; addr < (1 << 7); ++addr) {
            if(addr % 16 == 0) printf("%02x ", addr);

            // Perform a 1-byte dummy read from the probe address. If a slave
            // acknowledges this address, the function returns the number of bytes
            // transferred. If the address byte is ignored, the function returns
            // -1.

            // Skip over any reserved addresses.
            int ret;
            uint8_t rxdata;
            
            if(reservedAddress(addr)) ret = PICO_ERROR_GENERIC;
            else ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

            printf(ret < 0 ? "." : "@");
            printf(addr % 16 == 15 ? "\n" : "  ");
        }
        printf("Done.\n");
    } return;
}


void _saveToEEPROMTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);
        printf("Writing to EEPROM\r\n");
        
        uint8_t test = 42;
        i2c_write_blocking(i2c_default, 0xA1, &test, sizeof(test), true);
    } return;
}

void _readFromEEPROMTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);
        printf("Reading from EEPROM\r\n");

        uint8_t *test;
        i2c_read_blocking(i2c_default, 0xA0, test, sizeof(test), false);

        printf("From EEPROM: %d\r\n", *test);
    } return;
}


void _saveToFlashTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);
        if(xSemaphoreTake(g_vssDataMutex, 0) == pdTRUE) {
            printf("\033[21;1H");    // Move cursor to (21,1)
            printf("Start save routine for VSS     \r");
            
            uint8_t *p_vssAsBytes = (uint8_t*)&g_vss;
            uint writeSize = (sizeof(g_vss) / FLASH_PAGE_SIZE) + 1;
            uint sectorCount = ((writeSize * FLASH_PAGE_SIZE) / FLASH_SECTOR_SIZE) + 1;

            uint32_t interrupts = save_and_disable_interrupts();
                flash_range_erase(FLASH_VSS_OFFSET, FLASH_SECTOR_SIZE * sectorCount);
                flash_range_program(FLASH_VSS_OFFSET, p_vssAsBytes, FLASH_PAGE_SIZE * writeSize);
            restore_interrupts(interrupts);

            printf("\033[21;1H");    // Move cursor to (21,1)
            printf("End save routine for VSS     \r");

            xSemaphoreGive(g_vssDataMutex);
        }

        if(xSemaphoreTake(g_injDataMutex, 0) == pdTRUE) {
            printf("\033[22;1H");    // Move cursor to (21,1)
            printf("Start save routine for INJ");

            uint8_t *p_idAsBytes = (uint8_t*)&g_inj;
            uint writeSize = (sizeof(g_inj) / FLASH_PAGE_SIZE + 1);
            uint sectorCount = ((writeSize * FLASH_PAGE_SIZE) / FLASH_SECTOR_SIZE) + 1;

            uint32_t interrupts = save_and_disable_interrupts();
                flash_range_erase(FLASH_INJ_OFFSET, FLASH_SECTOR_SIZE * sectorCount);
                flash_range_program(FLASH_INJ_OFFSET, p_idAsBytes, FLASH_PAGE_SIZE * writeSize);
            restore_interrupts(interrupts);

            printf("\033[22;1H");    // Move cursor to (21,1)
            printf("End save routine for INJ");

            xSemaphoreGive(g_injDataMutex);
        }
    } return;
}
void _readFromFlashTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);
        uint32_t interrupts = save_and_disable_interrupts();

        if(xSemaphoreTake(g_vssDataMutex, 0) == pdTRUE) {
            VssData saveCheck;
            const uint8_t *p_vdFromFlash = (const uint8_t*)(XIP_BASE + FLASH_VSS_OFFSET);
            memcpy(&saveCheck, p_vdFromFlash + FLASH_PAGE_SIZE, sizeof(saveCheck));

            // We're looking for known flag in flash memory to know, if it's rubbish data or legit struct
            if(saveCheck.saveFlag == SAVE_FLAG_VAL) memcpy(&g_vss, p_vdFromFlash + FLASH_PAGE_SIZE, sizeof(g_vss));
            xSemaphoreGive(g_vssDataMutex);
        }

        if(xSemaphoreTake(g_injDataMutex, 0) == pdTRUE) {
            InjData saveCheck;
            
            const uint8_t *p_idFromFlash = (const uint8_t*)(XIP_BASE + FLASH_INJ_OFFSET);
            memcpy(&saveCheck, p_idFromFlash + FLASH_PAGE_SIZE, sizeof(saveCheck));

            // We're looking for known flag in flash memory to know, if it's rubbish data or legit struct
            if(saveCheck.saveFlag == SAVE_FLAG_VAL) memcpy(&g_inj, p_idFromFlash + FLASH_PAGE_SIZE, sizeof(g_inj));
            xSemaphoreGive(g_injDataMutex);
        } restore_interrupts(interrupts);
    } return;
}


// void initDataStructs() {}
void initIRQ(uint vssGPIO, uint injGPIO) {
    gpio_init(vssGPIO); 
    gpio_init(injGPIO);
    
    gpio_set_dir(vssGPIO, GPIO_IN); 
    gpio_set_dir(injGPIO, GPIO_IN);
    
    gpio_pull_up(vssGPIO); 
    gpio_pull_up(injGPIO);

    // Both IRQs use the same _ISR
    gpio_set_irq_enabled_with_callback(vssGPIO, GPIO_IRQ_EDGE_FALL, 1, _ISR);
    gpio_set_irq_enabled(injGPIO, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 1);

    return;
}
void _ISR(uint gpio) {
    BaseType_t checkYield;

    // Logic inside _ISR decides, which task should be resumed
    if(gpio == VSS_PIN) checkYield = xTaskResumeFromISR(g_vssTaskHandle);
    if(gpio == INJ_PIN) checkYield = xTaskResumeFromISR(g_injTaskHandle);

    portYIELD_FROM_ISR(checkYield);
    return;
}


void initTimer() {
    g_sysTimer = xTimerCreate("SYS TIMER", pdMS_TO_TICKS(SYS_TIMER_PERIOD_MS), pdTRUE, 0, _sysTimerISR);
    if(g_sysTimer == NULL) while(1);

    return;
}
void _sysTimerISR() {
    BaseType_t checkYield;
    checkYield = xTaskResumeFromISR(g_sysTimerTaskHandle);
    portYIELD_FROM_ISR(checkYield);

    return;
}

void _sysTimerTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);
        g_time++;

        if(g_time % 2 == 0)  vTaskResume(g_saveToEEPROMTaskHandle);
        else vTaskResume(g_readFromEEPROMTaskHandle);

        // if(xSemaphoreTake(g_vssDataMutex, 0) == pdTRUE) {
        //     // Current vehicle speed
        //     uint cs = g_pulseDistance * g_vss.distPulseCount * 3600;
        //     g_vss.currentSpeed = cs;
            
        //     // Average vehicle speed
        //     if(cs > 5) {
        //         if(xSemaphoreTake(g_injDataMutex, 0) == pdTRUE) {
        //             g_vss.rangeDistance = (uint)((g_inj.fuelLeft/g_inj.avgConsumption)*100);
        //             if(g_inj.insConsumption <= 0) g_vss.sailingDistance += g_pulseDistance;

        //             xSemaphoreGive(g_injDataMutex);
        //         } g_vss.avgSpeedDivider++;
                
        //         // Harmonic mean
        //         g_vss.sumInv += + (1.0f/(float)cs);
        //         g_vss.avgSpeed = g_vss.avgSpeedDivider/g_vss.sumInv;
        //     } 
            
        //     if(g_vss.saveFlag == 0) g_vss.saveFlag = SAVE_FLAG_VAL;
        //     xSemaphoreGive(g_vssDataMutex);
        // } if(uxQueueSpacesAvailable(g_vssDataQu) > 0) xQueueSend(g_vssDataQu, &g_vss, 0U);

        // if(xSemaphoreTake(g_injDataMutex, 0) == pdTRUE) {
        //     int ni = NO_INJ * 3600;  // no. of injectors * 3600 (seconds in hour)

        //     float iotv = (g_inj.openTime * g_injectionValue) * ni;
        //     float inv  = (g_inj.openTime * g_injectionValue) * NO_INJ;
        //     g_inj.openTime = ((float)g_inj.pulseTime/1000); // Converting to seconds

        //     if(xSemaphoreTake(g_vssDataMutex, 0) == pdTRUE) {
        //         if(g_vss.currentSpeed > 5) {
        //             float ic = (100*iotv)/(float)g_vss.currentSpeed;    // Consumption in liters per 100 kilometers
        //             g_inj.insConsumption = ic;

        //             // Harmonic mean
        //             if(ic > 0.0 && ic < 100.0) {
        //                 g_inj.sumInv += (1.0f/ic);
        //                 g_inj.avgConsumption = g_vss.avgSpeedDivider/g_inj.sumInv;
        //             }
        //         } else g_inj.insConsumption = iotv;                     // Consumption in liters per hour
        //         xSemaphoreGive(g_vssDataMutex);
        //     } 
            
        //     g_inj.fuelUsed += inv;
        //     g_inj.fuelLeft -= inv;
            
        //     if(g_inj.saveFlag == 0) g_inj.saveFlag = SAVE_FLAG_VAL;
        //     xSemaphoreGive(g_injDataMutex);
        // } if(uxQueueSpacesAvailable(g_injDataQu) > 0) xQueueSend(g_injDataQu, &g_inj, 0U);


        // // Read from flash on every boot
        // if(g_coldStart) vTaskResume(g_readFromFlashTaskHandle);
        // vTaskResume(g_displayTaskHandle);


        // if(xSemaphoreTake(g_vssDataMutex, 0) == pdTRUE) {         
        //     // If car is not moving and minute passed - save data to flash
        //     // if(g_vss.currentSpeed == 0 && g_time >= SAVE_INTERVAL) {
        //     //     xSemaphoreGive(g_vssDataMutex);
        //     //     vTaskResume(g_saveToFlashTaskHandle);
        //     // } 
            
        //     g_vss.distPulseCount = 0;
        //     xSemaphoreGive(g_vssDataMutex);
        // }

        // if(xSemaphoreTake(g_injDataMutex, 0) == pdTRUE) {
        //     // Save data to flash if car is idling and not moving
        //     // if(g_inj.pulseTime < 800 && g_time >= SAVE_INTERVAL) {
        //     //     xSemaphoreGive(g_injDataMutex);
        //     //     vTaskResume(g_saveToFlashTaskHandle);
        //     // } 
            
        //     g_inj.pulseTime = 0;
        //     xSemaphoreGive(g_injDataMutex);
        // }
    } return;
}


void _vssTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);     // NULL suspends itself
        if(xSemaphoreTake(g_vssDataMutex, 0) == pdTRUE) {
            g_vss.distPulseCount++;
            g_vss.traveledDistance += g_pulseDistance;
        } xSemaphoreGive(g_vssDataMutex);
    } return;
}


void _injTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);     // NULL suspends itself
        if(xSemaphoreTake(g_injDataMutex, 0) == pdTRUE) {
            if(gpio_get(INJ_PIN) == 0) g_inj.timeLow = xTaskGetTickCount();   // Low state on INJ_PIN
            else {
                // High state on INJ_PIN
                g_inj.timeHigh  = xTaskGetTickCount();
                g_inj.pulseTime = g_inj.pulseTime + ((g_inj.timeHigh/portTICK_PERIOD_MS) - (g_inj.timeLow/portTICK_PERIOD_MS));
                gpio_put(INJ_PIN, 0);
            } xSemaphoreGive(g_injDataMutex);
        }
    } return;
}


void _displayTask(void *params) {
    VssData vd; InjData id;

    while(1) {
        vTaskSuspend(NULL);
        if(g_coldStart) {
            printf("\e[1;1H\e[2J");     // Clear screen at boot
            g_coldStart = 0;
        }

        if(uxQueueMessagesWaiting(g_vssDataQu) > 0) {
            xQueueReceive(g_vssDataQu, &vd, portMAX_DELAY);

            printf("\033[1;1H");    // Move cursor to (1, 1)
            printf("Cur speed: %d  \r", vd.currentSpeed);
        
            printf("\033[2;1H");    // Move cursor to (2, 1)
            printf("Avg speed: %d  \r", vd.avgSpeed);

            printf("\033[3;1H");    // Move cursor to (3, 1)
            printf("Range: %d  \r", vd.rangeDistance);

            printf("\033[4;1H");    // Move cursor to (4, 1)
            printf("Travelled distance: %.1f  \r", vd.traveledDistance);

            printf("\033[5;1H");    // Move cursor to (5, 1)
            printf("Sailing distance: %.1f  \r", vd.sailingDistance);
        }

        if(uxQueueMessagesWaiting(g_injDataQu) > 0) {
            xQueueReceive(g_injDataQu, &id, portMAX_DELAY);

            printf("\033[11;1H");    // Move cursor to (11,1)
            printf("Ins fuel cons: %.1f  \r", id.insConsumption);

            printf("\033[12;1H");    // Move cursor to (12,1)
            printf("Avg fuel cons: %.1f  \r", id.avgConsumption);

            printf("\033[13;1H");    // Move cursor to (13,1)
            printf("Fuel left: %.1f  \r", id.fuelLeft);

            printf("\033[14;1H");    // Move cursor to (14,1)
            printf("Fuel used: %.1f  \r", id.fuelUsed);

            printf("\033[15;1H");    // Move cursor to (15,1)
            printf("Fuel saved: %.1f  \r", id.fuelSaved);
        }

        // Debug
        printf("\033[20;1H");    // Move cursor to (20,1)
        printf("Time: %ds  \r", g_time);

        printf("\033[21;1H");    // Move cursor to (21,1)
        printf("Sizeof vss struct: %dB  \r", sizeof(vd));

        printf("\033[22;1H");    // Move cursor to (21,1)
        printf("Sizeof inj struct: %dB  \r", sizeof(id));
    } return;
}


// I2C Bus Scanner
//    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
// 00 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 10 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 20 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 30 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 40 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 50 @  @  @  @  @  @  @  @  .  .  .  .  .  .  .  .
// 60 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// 70 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
// Done.
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"


void ledTask(void *params);
void statusTask(void *params);

static QueueHandle_t ledStatus = NULL;

int main() {
    stdio_init_all();
    // if(cyw43_arch_init()) {
    //     printf("WiFi init failed");
    //     return -1;
    // } printf("WiFi init successfull");


    ledStatus = xQueueCreate(1, sizeof(uint));

    xTaskCreate(statusTask, "Status Task",
        256, NULL,
        1, NULL
    );

    xTaskCreate(ledTask, "LED Task",
        256, NULL,
        1, NULL
    );


    vTaskStartScheduler();
    while(1) {}

    // cyw43_arch_deinit();
    return 0;
}


void ledTask(void *params) {
    uint state = 0;
    
    while(1) {
        // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
            state = 1;
            xQueueSend(ledStatus, &state, 0U);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
            state = 0;
            xQueueSend(ledStatus, &state, 0U);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    } return;
}

void statusTask(void *params) {
    uint state = 0;

    while(1) {
        xQueueReceive(ledStatus, &state, portMAX_DELAY);

        if(state) printf("LED is ON\r\n");
        else printf("LED is OFF\r\n");
    } return;
}
#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"


void ledTask(void *params);

int main() {
    stdio_init_all();
    if(cyw43_arch_init()) {
        printf("WiFi init failed");
        return -1;
    }

    xTaskCreate(ledTask, "LED Task",
        256, NULL,
        1, NULL
    );

    vTaskStartScheduler();
    while(1) {}

    cyw43_arch_deinit();
    return 0;
}


void ledTask(void *params) {
    while(1) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    } return;
}
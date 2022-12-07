#ifndef VSS_PIN
#define VSS_PIN 22
#endif

static float g_pulseDistance  = 0.00006823;

typedef struct {
    uint avgSpeed;
    uint currentSpeed;
    uint distPulseCount;

    float sumInv;               // For harmonic mean calculations
    float avgSpeedDivider;
    float traveledDistance;
} VssData; VssData g_vss = {0};

static QueueHandle_t g_vssDataQu = NULL;
static TaskHandle_t g_vssTaskHandle = NULL;
static SemaphoreHandle_t g_vssDataMutex = NULL;


void _vssTask(void *params) {
    while(1) {
        vTaskSuspend(NULL);     // NULL suspends itself
        if(xSemaphoreTake(g_vssDataMutex, 0) == pdTRUE) {
            g_vss.distPulseCount++;
            g_vss.traveledDistance += g_pulseDistance;
        } xSemaphoreGive(g_vssDataMutex);
    } return;
}

#include <stdio.h>

#include "esp32plus/semaphore.h"


Semaphore::~Semaphore() {
    vSemaphoreDelete(semaphore);
}

IRAM_ATTR BaseType_t Semaphore::take(TickType_t ticks_to_wait) {
    return xSemaphoreTake(semaphore, ticks_to_wait) == pdTRUE;
}

IRAM_ATTR BaseType_t Semaphore::give() {
    return xSemaphoreGive(semaphore);
}


Mutex::Mutex() {
    semaphore = xSemaphoreCreateMutex();
}



#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


class Semaphore {
    public:
        ~Semaphore();

        IRAM_ATTR bool take(TickType_t ticks_to_wait = 0);
        IRAM_ATTR bool give();

    protected:
        SemaphoreHandle_t semaphore = NULL;
};


class Mutex : public Semaphore {
    public:
        Mutex();
};

#pragma once

#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "state.h"

void uiPrepareDisplayBoot();
bool uiStart(SharedState* shared_state, SemaphoreHandle_t state_mutex, QueueHandle_t command_queue);

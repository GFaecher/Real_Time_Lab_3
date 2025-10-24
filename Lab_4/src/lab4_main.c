// Include FreeRTOS headers.
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include "timers.h"
#include "stm32l4xx.h"
#include "stm32l432xx.h"
#include <stdbool.h>
#include "lib_ee152.h"

// Write a sawtooth out on DAC 1, which drives PA3.
// The spec: frequency=100Hz, amplitude of roughly 3.3V.
// Remember that 100Hz is ?? FreeRTOS ticks, and 3.3V is an argument of ??
// to analogWrite().
void task_sawtooth (void * pvParameters) {
    uint32_t value = 0;
    while(1){
        analogWrite(A3, value);
        analogWrite(A4, value);
        value = (value + 25);
        if(value >= 250) {
                value = 0;
        }
        vTaskDelay(1);
    }
}

//every 10ms 2550 ms = 2.55s
// .01s / 51 = 0.039 vtask delay 

#define BLINK_GREEN_DELAY ( 500 / portTICK_PERIOD_MS )
void task_blink_green (void *pvParameters) {
        bool is_on = 0;
        while(1) {
                digitalWrite(D13, is_on);
                is_on = !is_on;
                vTaskDelay(250);
        }
}

int main() {
    //clock_setup_16MHz();		// 16 MHz, AHB and APH1/2 prescale=1x
    clock_setup_80MHz();		// 80 MHz, AHB and APH1/2 prescale=1x

    // The green LED is at Nano D13, or PB3.
    pinMode(D13, "OUTPUT");
    pinMode(A3, "OUTPUT");
    pinMode(A4, "OUTPUT");
    digitalWrite (D13, 0);

    // Create tasks.

    TaskHandle_t task_handle_sawtooth = NULL;
    BaseType_t task_create_OK = xTaskCreate (
	    task_sawtooth, "sawtooth gen",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    1+tskIDLE_PRIORITY, // priority
	    &task_handle_sawtooth);
    if (task_create_OK != pdPASS) for ( ;; );

    // Next, writer task #1, that just writes.
    TaskHandle_t task_handle_blink_green = NULL;
    task_create_OK = xTaskCreate (
	    task_blink_green, "blink green",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY, // priority
	    &task_handle_blink_green);
    if (task_create_OK != pdPASS) for ( ;; );

    vTaskStartScheduler();
}
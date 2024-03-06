#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"

// Define pin numbers for the signals and buttons
#define DATA 15
#define SYNC 21
#define LED_PIN1 22
#define LED_PIN2 23

// Define constants for signal generation based on the defined timing
#define TIME_DELAY 100 // Adjust based on your operational mode
#define A (2 * TIME_DELAY)
#define B (1 * TIME_DELAY)
#define C 17
#define D (35 * TIME_DELAY)
#define SYNC_ON 50

// Initialize global variables to manage output and selection states
bool OUTPUT_STATE = false;
bool SELECT_STATE = false;
bool prevButton1State = false;
bool prevButton2State = false;

// Function Prototypes
void setup_gpio();
void loop_task(void *pvParameter);
void triggerSyncSignal();
void generateSignals(int a, char c, int b, int d);
void generateDataPulse(int pulseDuration, int intervalDuration);

void setup_gpio() {
    gpio_pad_select_gpio(DATA);
    gpio_set_direction(DATA, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(SYNC);
    gpio_set_direction(SYNC, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(LED_PIN1);
    gpio_set_direction(LED_PIN1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LED_PIN1, GPIO_PULLDOWN_ONLY);

    gpio_pad_select_gpio(LED_PIN2);
    gpio_set_direction(LED_PIN2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LED_PIN2, GPIO_PULLDOWN_ONLY);
}


void triggerSyncSignal() {
    gpio_set_level(SYNC, 1);
    ets_delay_us(SYNC_ON);
    gpio_set_level(SYNC, 0);
}

void generateSignals(int a, char c, int b, int d) {
    triggerSyncSignal();
    for (char i = 0; i < c; i++) {
        generateDataPulse(a, b);
        a += TIME_DELAY;
    }
    ets_delay_us(d);
}

void generateDataPulse(int pulseDuration, int intervalDuration) {
    gpio_set_level(DATA, 1);
    ets_delay_us(pulseDuration);
    gpio_set_level(DATA, 0);
    ets_delay_us(intervalDuration);
}


void app_main() {
    setup_gpio();
    while(1) {
        bool currentButton1State = gpio_get_level(LED_PIN1);
        bool currentButton2State = gpio_get_level(LED_PIN2);

        if (currentButton1State != prevButton1State && currentButton1State == 1) {
            vTaskDelay(50 / portTICK_PERIOD_MS); // Debounce delay
            OUTPUT_STATE = !OUTPUT_STATE;
            ESP_LOGI("OUTPUT_STATE", OUTPUT_STATE ? "Signal ON" : "Signal OFF");
        }

        if (currentButton2State != prevButton2State && currentButton2State == 1) {
            vTaskDelay(50 / portTICK_PERIOD_MS); // Debounce delay
            SELECT_STATE = !SELECT_STATE;
            ESP_LOGI("SELECT_STATE", SELECT_STATE ? "Mode4 ON" : "Mode4 OFF");
        }

        prevButton1State = currentButton1State;
        prevButton2State = currentButton2State;

        if (OUTPUT_STATE) {
            int a = A, b = (SELECT_STATE ? B / 2 : B), d = (SELECT_STATE ? D / 2 : D);
            generateSignals(a, C, b, d);
        } else {
            gpio_set_level(DATA, 0);
            gpio_set_level(SYNC, 0);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Short delay to avoid WDT reset
    }

}


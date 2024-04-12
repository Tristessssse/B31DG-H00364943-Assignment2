#include <Arduino.h>

// Define GPIO pin numbers for various components
#define DIGITAL_SIGNAL_PIN     23  // Output pin for digital signal
#define ANALOG_INPUT_PIN1      33  // Input pin for first analog sensor
#define ANALOG_INPUT_PIN2      32  // Input pin for second analog sensor
#define LED_PIN1               15  // LED output pin for visual feedback
#define LED_PIN2               21  // Second LED output pin
#define BUTTON_PIN             22  // Input pin for pushbutton
#define ANALOG_READ_PIN        25  // Analog input pin for reading signals
#define DEBOUNCE_TIME          50  // Debounce time for button press in milliseconds

// Structure to store and manage all shared resources across different tasks
struct SystemResources {
    SemaphoreHandle_t freqSemaphore;   // Semaphore for protecting frequency data access
    QueueHandle_t eventQueue;          // Queue for inter-task communication
    volatile unsigned long lastRisingTime = 0;   // Last timestamp when a rising edge was detected
    volatile unsigned long period = 0;           // Period between consecutive rising edges
    volatile bool newPeriodAvailable = false;    // Flag to indicate new period data is available
    volatile unsigned long lastRisingTime2 = 0;  // As above, for second measurement point
    volatile unsigned long period2 = 0;          
    volatile bool newPeriodAvailable2 = false;   
    volatile int freq1 = 0;                      // Calculated frequency from first point
    volatile int freq2 = 0;                      // Calculated frequency from second point
};

SystemResources resources;  // Global instance of resources

// Function to scale frequency value to a range of 0 to 99 based on predefined frequency limits
int scaleFrequency(int frequency, int minFreq, int maxFreq) {
    if (frequency <= minFreq) return 0;
    if (frequency >= maxFreq) return 99;
    return (frequency - minFreq) * 99 / (maxFreq - minFreq);
}

// Interrupt service routine for detecting rising edges at the first sensor
void IRAM_ATTR onSquareWaveRisingEdge() {
    unsigned long currentTime = micros();
    if (resources.lastRisingTime != 0) {
        unsigned long currentPeriod = currentTime - resources.lastRisingTime;
        resources.period = currentPeriod;
        resources.newPeriodAvailable = true;
    }
    resources.lastRisingTime = currentTime;
}

// As above, but for the second sensor
void IRAM_ATTR onSquareWaveRisingEdge2() {
    unsigned long currentTime = micros();
    if (resources.lastRisingTime2 != 0) {
        unsigned long currentPeriod = currentTime - resources.lastRisingTime2;
        resources.period2 = currentPeriod;
        resources.newPeriodAvailable2 = true;
    }
    resources.lastRisingTime2 = currentTime;
}

void setup() {
    Serial.begin(9600);
    // Setup pin modes for inputs and outputs
    pinMode(DIGITAL_SIGNAL_PIN, OUTPUT);
    pinMode(ANALOG_INPUT_PIN1, INPUT);
    pinMode(ANALOG_INPUT_PIN2, INPUT);
    pinMode(LED_PIN1, OUTPUT);
    pinMode(LED_PIN2, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(ANALOG_READ_PIN, INPUT);

    // Create a mutex for managing access to shared resources across tasks
    resources.freqSemaphore = xSemaphoreCreateMutex();
    if (resources.freqSemaphore == NULL) {
        Serial.println("Failed to create the semaphore");
        while(1);
    }

    // Create an event queue for task communication
    resources.eventQueue = xQueueCreate(10, sizeof(int));
    if (resources.eventQueue == NULL) {
        Serial.println("Failed to create the queue");
        while(1);
    }

    // Attach interrupts to handle frequency measurement
    attachInterrupt(digitalPinToInterrupt(ANALOG_INPUT_PIN1), onSquareWaveRisingEdge, RISING);
    attachInterrupt(digitalPinToInterrupt(ANALOG_INPUT_PIN2), onSquareWaveRisingEdge2, RISING);

    // Initialize tasks for various functions
    xTaskCreate(digitalSignalTask, "Digital Signal Output", 1024, &resources, 1, NULL);
    xTaskCreate(taskMeasureFrequency1, "Measure Frequency", 2048, &resources, 2, NULL);
    xTaskCreate(taskMeasureFrequency2, "Measure Frequency", 2048, &resources, 2, NULL);
    xTaskCreate(sampleAndAverageAnalogInputTask, "Sample And Average Analog Input", 1024, &resources, 2, NULL);
    xTaskCreate(logFrequencyTask, "Log Frequency", 2048, &resources, 1, NULL);
    xTaskCreate(buttonTask1, "Button Control LED", 1024, &resources, 2, NULL);
    xTaskCreate(controlLedTask, "Control LED Task", 1024, &resources, 2, NULL);
    xTaskCreate(workloadTask, "CPU Workload", 2048, &resources, 1, NULL);
}

void loop() {
    // Main loop is empty because FreeRTOS handles the loop internally.
}

// Task to generate a periodic digital signal
void digitalSignalTask(void *pvParameters) {
    SystemResources *res = static_cast<SystemResources*>(pvParameters);
    while (1) {
        digitalWrite(DIGITAL_SIGNAL_PIN, HIGH);
        delayMicroseconds(180);
        digitalWrite(DIGITAL_SIGNAL_PIN, LOW);
        delayMicroseconds(40);
        digitalWrite(DIGITAL_SIGNAL_PIN, HIGH);
        delayMicroseconds(530);
        digitalWrite(DIGITAL_SIGNAL_PIN, LOW);
        delayMicroseconds(250);
        vTaskDelay(3 / portTICK_PERIOD_MS);
    }
}

// Task to measure frequency based on the time between rising edges
void taskMeasureFrequency1(void *pvParameters) {
    SystemResources *res = static_cast<SystemResources*>(pvParameters);
    while (1) {
        if (res->newPeriodAvailable) {
            xSemaphoreTake(res->freqSemaphore, portMAX_DELAY);
            res->freq1 = 1000000.0 / res->period; // Convert period to frequency
            res->newPeriodAvailable = false;
            xSemaphoreGive(res->freqSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Task to measure frequency at a second point
void taskMeasureFrequency2(void *pvParameters) {
    SystemResources *res = static_cast<SystemResources*>(pvParameters);
    while (1) {
        if (res->newPeriodAvailable2) {
            xSemaphoreTake(res->freqSemaphore, portMAX_DELAY);
            res->freq2 = 1000000.0 / res->period2; // Convert period to frequency
            res->newPeriodAvailable2 = false;
            xSemaphoreGive(res->freqSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(8));
    }
}

// Task to sample analog input and compute a running average
void sampleAndAverageAnalogInputTask(void *pvParameters) {
    SystemResources *res = static_cast<SystemResources*>(pvParameters);
    int readings[10] = {0}; // Buffer for last 10 readings
    int total = 0;
    int readIndex = 0;
    int average = 0;

    while(1) {
        int readValue = analogRead(ANALOG_READ_PIN);
        total -= readings[readIndex];  // Remove the oldest reading
        readings[readIndex] = readValue;  // Insert the new reading
        total += readings[readIndex];  // Add the new reading
        readIndex = (readIndex + 1) % 10;  // Move to the next index in the circular buffer

        average = total / 10;  // Calculate average of readings

        // Condition to toggle LED based on the average value
        if (average > 2047) {
            digitalWrite(LED_PIN1, HIGH);
        } else {
            digitalWrite(LED_PIN1, LOW);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Task to log the scaled frequency values
void logFrequencyTask(void *pvParameters) {
    SystemResources *res = static_cast<SystemResources*>(pvParameters);
    while (1) {
        xSemaphoreTake(res->freqSemaphore, portMAX_DELAY);
        int scaledFreq1 = scaleFrequency(res->freq1, 333, 1000);  // Scale frequency to a 0-99 range
        int scaledFreq2 = scaleFrequency(res->freq2, 500, 1000);
        xSemaphoreGive(res->freqSemaphore);

        Serial.printf("Scaled Frequency 1: %d, Scaled Frequency 2: %d\n", scaledFreq1, scaledFreq2);
        vTaskDelay(pdMS_TO_TICKS(200)); // Log every 200ms
    }
}

// Task to handle button presses and control LED state
void buttonTask1(void *pvParameters) {
    SystemResources *res = static_cast<SystemResources*>(pvParameters);
    static uint32_t lastDebounceTime = 0;
    static bool lastButtonState = HIGH;
    bool reading;

    while(1) {
        reading = digitalRead(BUTTON_PIN);
        
        // Debounce logic to ensure stable button press reading
        if (reading != lastButtonState) {
            lastDebounceTime = millis();
        }
        if ((millis() - lastDebounceTime) > DEBOUNCE_TIME && reading != lastButtonState) {
            lastButtonState = reading;
            if (reading == LOW) {
                int event = 1;
                xQueueSend(res->eventQueue, &event, portMAX_DELAY);  // Send button press event to queue
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10ms
    }
}

// Task to control the LED based on button presses
void controlLedTask(void *pvParameters) {
    SystemResources *res = static_cast<SystemResources*>(pvParameters);
    int receivedEvent;
    static bool ledState = LOW;

    while(1) {
        if (xQueueReceive(res->eventQueue, &receivedEvent, portMAX_DELAY) == pdTRUE) {
            if (receivedEvent == 1) {  // If button press event received
                ledState = !ledState;  // Toggle LED state
                digitalWrite(LED_PIN2, ledState);
            }
        }
    }
}

// Simulate CPU workload by consuming processing time
void CPU_work(int time) {
    long endTime = millis() + time;
    while(millis() < endTime) {
        // Empty loop to consume time
    }
}

// Task to periodically simulate CPU workload
void workloadTask(void *pvParameters) {
    SystemResources *res = static_cast<SystemResources*>(pvParameters);
    while(1) {
        CPU_work(2);  // Simulate CPU workload for 2ms
        vTaskDelay(pdMS_TO_TICKS(20) - pdMS_TO_TICKS(2)); // Ensure total cycle is about 20ms
    }
}

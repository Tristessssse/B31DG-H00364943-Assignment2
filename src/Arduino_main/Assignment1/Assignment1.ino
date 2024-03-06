// Define operational mode. Uncomment the line below for production mode.
#define PRODUCTION_MODE

// Set timing based on operational mode
#ifdef PRODUCTION_MODE
#define TIME_DELAY 100 // Shorter delay for production
#else
#define TIME_DELAY 100000 // Longer delay for debugging
#endif

// Assign pin numbers for the signals and buttons
#define DATA 15 // Pin for the DATA signal
#define SYNC 21   // Pin for the SYNC signal
#define LED_PIN1 22 // Pin for button 1 (output state toggle)
#define LED_PIN2 23 // Pin for button 2 (select mode toggle)

// Define constants for signal generation based on the defined timing
#define A (2 * TIME_DELAY) // Duration of each DATA pulse
#define B (1 * TIME_DELAY) // Interval between DATA pulses
#define C 17 // Total number of pulses in a signal sequence
#define D (35 * TIME_DELAY) // Delay after a complete signal sequence

#define SYNC_ON 50 // Duration the SYNC signal stays HIGH

// Initialize global variables to manage output and selection states
bool OUTPUT_STATE = false; // Tracks whether output is enabled
bool SELECT_STATE = false; // Tracks which signal mode is selected
// Variables to store previous button states for edge detection
bool prevButton1State = false;
bool prevButton2State = false;

void setup() {
  Serial.begin(115200); // Initialize serial communication
  // Configure pins for buttons and signals
  pinMode(LED_PIN1, INPUT_PULLDOWN);
  pinMode(LED_PIN2, INPUT_PULLDOWN);
  pinMode(DATA, OUTPUT);
  pinMode(SYNC, OUTPUT);
}

void loop() {
  // Read the current state of each button
  bool currentButton1State = digitalRead(LED_PIN1);
  bool currentButton2State = digitalRead(LED_PIN2);

  // Toggle the output state on button 1 press
  if (currentButton1State != prevButton1State && currentButton1State == HIGH) {
    delay(50); // Debounce delay
    OUTPUT_STATE = !OUTPUT_STATE; // Toggle the state
    Serial.println(OUTPUT_STATE ? "Signal ON" : "Signal OFF"); // Log state change
  }

  // Toggle the selection state on button 2 press
  if (currentButton2State != prevButton2State && currentButton2State == HIGH) {
    delay(50); // Debounce delay
    SELECT_STATE = !SELECT_STATE; // Toggle the mode
    Serial.println(SELECT_STATE ? "Mode4 ON" : "Mode4 OFF"); // Log mode change
  }

  // Update previous button states for the next iteration
  prevButton1State = currentButton1State;
  prevButton2State = currentButton2State;

  // Based on the output state, either generate signals or disable output
  if (OUTPUT_STATE) {
    // Adjust signal parameters based on the selection state
    int a = A, b = (SELECT_STATE ? B / 2 : B), d = (SELECT_STATE ? D / 2 : D);
    generateSignals(a, C, b, d); // Generate the signal sequence
  } else {
    // Disable signal output
    digitalWrite(DATA, LOW);
    digitalWrite(SYNC, LOW);
  }
}

// Function to trigger the SYNC signal
void triggerSyncSignal() {
  digitalWrite(SYNC, HIGH);
  delayMicroseconds(SYNC_ON);
  digitalWrite(SYNC, LOW);
}

// Function to generate the DATA pulse sequence
void generateSignals(int a, char c, int b, int d) {
  triggerSyncSignal(); // Activate SYNC signal
  for (char i = 0; i < c; i++) {
    // Generate each DATA pulse and interval
    generateDataPulse(a, b);
    a += TIME_DELAY; // Increment pulse duration for the next pulse
  }
  delayMicroseconds(d); // Delay after completing the sequence
}

// Function to generate a single DATA pulse
void generateDataPulse(int pulseDuration, int intervalDuration) {
  digitalWrite(DATA, HIGH);
  delayMicroseconds(pulseDuration);
  digitalWrite(DATA, LOW);
  delayMicroseconds(intervalDuration);
}

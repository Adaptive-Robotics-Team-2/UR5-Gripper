#include "Adafruit_ZeroTimer.h"

// Timer tester
Adafruit_ZeroTimer zt3 = Adafruit_ZeroTimer(3);

unsigned long lastInterruptTime = 0;  // Store the last interrupt time

// Define the interrupt handler
void TC3_Handler() {
  Adafruit_ZeroTimer::timerHandler(3);
}

// The timer 3 callback
void Timer3Callback0() {
  // Get the current time in milliseconds
  unsigned long currentTime = millis();
  
  // Calculate the time difference since the last interrupt
  unsigned long timeDifference = currentTime - lastInterruptTime;
  
  // Print the time difference in milliseconds
  Serial.print("Time between interrupts: ");
  Serial.print(timeDifference);
  Serial.println(" ms");
  
  // Update the last interrupt time to the current time
  lastInterruptTime = currentTime;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial connection
  Serial.println("Setting up timer");

  #define INTERVAL 1000  // Interval in milliseconds
  
  // Convert interval to seconds
  float intervalSec = INTERVAL / 1000.0;
  
  // Calculate the compare value using the formula
  unsigned long compareValue = 46874 * intervalSec;
  
  // Print the calculated compare value for debugging
  Serial.print("Compare Value: ");
  Serial.println(compareValue);

  /********************* Timer #3, 16-bit, INTERVAL period */
  zt3.configure(TC_CLOCK_PRESCALER_DIV1024, // prescaler
                TC_COUNTER_SIZE_16BIT,      // bit width of timer/counter
                TC_WAVE_GENERATION_MATCH_PWM); // match style

  // Set compare value
  zt3.setCompare(0, compareValue);

  // Set the callback to be triggered at the defined interval
  zt3.setCallback(true, TC_CALLBACK_CC_CHANNEL0, Timer3Callback0);

  // Enable the timer
  zt3.enable(true);

  // Initialize lastInterruptTime
  lastInterruptTime = millis();
}

void loop() {
  // Main loop does nothing, timer runs in the background
}

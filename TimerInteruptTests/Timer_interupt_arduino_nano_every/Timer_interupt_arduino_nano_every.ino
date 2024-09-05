#define INTERRUPT_INTERVAL_MS 500 // 0.5 seconds

void setup() {
  // Set up the timer
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc; // Set clock source to system clock
  TCB0.CCMP = (F_CPU / 1000) * INTERRUPT_INTERVAL_MS - 1; // Set compare match value for 0.5 seconds
  TCB0.INTCTRL = TCB_CAPT_bm; // Enable interrupt on compare match
  TCB0.CTRLA |= TCB_ENABLE_bm; // Enable the timer

  // Set up serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Main loop does nothing, interrupt handles timing
}

// Timer interrupt service routine
ISR(TCB0_INT_vect) {
  // Clear the interrupt flag
  TCB0.INTFLAGS = TCB_CAPT_bm;
  
  // Print message
  Serial.println("Timer interrupt triggered");
}

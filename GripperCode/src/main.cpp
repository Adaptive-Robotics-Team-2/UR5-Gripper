#include <Arduino.h>
#include <Servo.h>
// #include "Adafruit_ZeroTimer.h"

#pragma region TimerVariables

// // Timer tester
// Adafruit_ZeroTimer zt3 = Adafruit_ZeroTimer(3);

// unsigned long lastInterruptTime = 0; // Store the last interrupt time

// #define INTERVAL 300 // Interval in milliseconds

// // Convert interval to seconds
// float intervalSec = INTERVAL / 1000.0;

// // Calculate the compare value using the formula
// unsigned long compareValue = 46874 * intervalSec;

#pragma endregion

// Todo
// CHeck pinout for UR5
#pragma region PinDefinitionsUR5

// define the arduino pins
#define BOOT_PIN 1
#define TRIG_PIN 6

// define the UR5 pins
#define ECHO_CLOSE_PIN 3
#define ECHO_PIN 5

#pragma endregion

#pragma region PressureSensorDefinitions

const int PRESSURE_PIN = A0; // Analog pin for pressure sensor

int currentPressureValue = 0; // Variable to store pressure sensor value

const int pressureDroppedThreshold = 0;   // Threshold value indicating an object is dropped
const int pressureGripThreshold = 500;    // Threshold value indicating an object is picked up
const int pressureMaximumThreshold = 800; // Threshold value indicating Gripper is being to agressive

bool gripperIsOpen = false; // Flag to indicate if an object is picked up

const int resetPin = 1;

#pragma endregion

#pragma region ServoDefinitions
const int SERVO_PIN = 11; // Digital pin for servo motor

Servo gripperServo; // Servo object for the gripper

const int openPosition = 0; // Gripper fully open
int currentPosition = 0;    // Current servo position, starts at 0
int closePosition = 0;      // Variable to store the position where the object is gripped

const int openingPosition = 0;  // Position to open the gripper
const int closingPosition = 45; // Position to close the gripper

#pragma endregion

#pragma region ResetButtonDefinitions
#define BOOT_BUTTON 11

int buttonState = 0;

#pragma endregion

#pragma region StateDefinitions

enum GripperState
{
    BOOT,
    GRIPPER_IDLE_MOVING_UR5,
    MOVE_GRIPPERS,
    STOP
};

GripperState currentGripperState = BOOT;

#pragma endregion

bool programIsRunning = false;

// function prototypes
bool sendPulse(int pin);
int readPulse(int pin);
void initGripper();
void initPressureSensorInterrupt();
void initControlPins();
void openGripper();
void closeGripper();

// // Define the interrupt handler
// void TC3_Handler()
// {
//     Adafruit_ZeroTimer::timerHandler(3);
// }

// // The timer 3 callback
// void Timer3Callback0()
// {
//     pressureValue = analogRead(PRESSURE_PIN);

//     //?Stop servo if pressure exceeds threshold
//     if (pressureValue > pressureMaximumThreshold)
//     {
//         gripperServo.write(0);
//     }
// }

#pragma region InitFunctions
void initControlPins()
{
    pinMode(BOOT_PIN, OUTPUT);
    pinMode(ECHO_CLOSE_PIN, INPUT);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(BOOT_BUTTON, INPUT_PULLUP);
}

void initGripper()
{
    Serial.println("Initializing Gripper...");
    if (!gripperServo.attach(SERVO_PIN))
    {
        Serial.println("Failed to attach servo, stopping the robot.");
        currentGripperState = STOP;
        return;
    }

    // Move the servo from 0 to 50 degrees in steps of 5 degrees
    for (int position = 0; position <= 110; position += 5)
    {
        gripperServo.write(position);
        delay(100); // Wait for the servo to reach the position

        // Debug
        if (currentPressureValue > pressureMaximumThreshold)
        {
            Serial.print("Pressure exceeded threshold at position: ");
            Serial.println(position);
            currentGripperState = STOP;
            break;
        }
    }
    gripperServo.write(0);
}

// void initPressureSensorInterrupt()
// {
//     /********************* Timer #3, 16-bit, INTERVAL period */
//     zt3.configure(TC_CLOCK_PRESCALER_DIV1024,    // prescaler
//                   TC_COUNTER_SIZE_16BIT,         // bit width of timer/counter
//                   TC_WAVE_GENERATION_MATCH_PWM); // match style

//     // Set compare value
//     zt3.setCompare(0, compareValue);

//     // Set the callback to be triggered at the defined interval
//     zt3.setCallback(true, TC_CALLBACK_CC_CHANNEL0, Timer3Callback0);

//     // Enable the timer
//     zt3.enable(true);

//     // Initialize lastInterruptTime
//     lastInterruptTime = millis();
// }
#pragma endregion

// TODO add a timeout or strikecount
bool sendPulse(int pin)
{
    // Set the specified pin high
    digitalWrite(pin, HIGH);
    Serial.print("Pin ");
    Serial.print(pin);
    Serial.println(" set HIGH");

    Serial.print("Waiting for response on pin ");
    Serial.println(ECHO_PIN);
    // Wait for response on pin ECHO_PIN
    while (readPulse(ECHO_PIN) != HIGH)
    {
    }
    Serial.print("Response received on pin ");
    Serial.println(ECHO_PIN);
    
    digitalWrite(pin, LOW); // Set the pin low again
    // Return true when pin 2 is high
    return true;
}

int readPulse(int pin)
{
    return digitalRead(pin);
}

void openGripper()
{
    Serial.println("Opening Grippers to the position where object was gripped...");
    while (currentPosition > openPosition)
    {
        currentPosition -= 5;
        gripperServo.write(currentPosition);
        delay(50);
        currentPressureValue = analogRead(PRESSURE_PIN);
        if (currentPressureValue >= pressureMaximumThreshold)
        {
            Serial.print("ERROR");
            Serial.println(currentPosition);
            closePosition = currentPosition; // Store the position where the object was gripped
            currentGripperState = STOP;
            break;
        }

        if (currentPosition <= openPosition)
        {
            gripperIsOpen = true;
            Serial.println("Gripper fully opened.");
            break;
        }
    }
    Serial.println("Gripper Open...");
}

void closeGripper()
{
    Serial.println("Closing Grippers...");
    while (currentPressureValue <= pressureGripThreshold && currentPosition < 120)
    { // Allow movement up to 120 degrees
        currentPosition += 10;
        gripperServo.write(currentPosition);
        delay(100);

        currentPressureValue = analogRead(PRESSURE_PIN);
        if (currentPressureValue >= pressureMaximumThreshold)
        {
            Serial.print("ERROR");
            Serial.println(currentPosition);
            closePosition = currentPosition; // Store the position where the object was gripped
            currentGripperState = STOP;
            break;
        }
    }

    //! Now when ERROR occured it whil stil print the below object grabbed
    Serial.println("Object grabbed...");
    gripperIsOpen = false;
}

void setup()
{
    Serial.begin(9600);

    initControlPins();
    // initGripper();
    // initPressureSensorInterrupt();
    Serial.println("Setup complete...");
}

void loop()
{
    buttonState = digitalRead(BOOT_BUTTON);

    if (buttonState == HIGH && programIsRunning == false)
    {
        Serial.println("Booting up the robot again");
        currentGripperState = BOOT;
    }
    else if (buttonState == HIGH && programIsRunning == true)
    {
        Serial.println("Stopping the robot");
        currentGripperState = STOP;
    }

    // when the robot is done with all its steps, it will stop and wait for the button to be pressed
    if (readPulse(ECHO_CLOSE_PIN) == HIGH)
    {
        Serial.println("Robot is done with all its steps, waiting for the button to be pressed to rerstart the program");
        currentGripperState = STOP;
    }

    switch (currentGripperState)
    {
    case BOOT:
        // send a pulse to the boot pin
        Serial.println("Booting up the robot");

        // UR5 should send messege when it is ready to use (at its starting position)
        if (sendPulse(BOOT_PIN))
        {
            programIsRunning = true;
            currentGripperState = GRIPPER_IDLE_MOVING_UR5;
            break;
        }
    case GRIPPER_IDLE_MOVING_UR5:
        Serial.println("Gripper is idle");

        // Send pulse so the UR5 moves to its next location.
        // UR5 sends message back when it arrives
        if (sendPulse(TRIG_PIN))
        {
            Serial.println("UR5 moved to next location");

            // gripperIsOpen = !gripperIsOpen; // toggle the gripper currentGripperState when the arm is done moving
            currentGripperState = MOVE_GRIPPERS;
        }

    case MOVE_GRIPPERS:
        // based on the gripper currentGripperState, open or close the gripper
        if (!gripperIsOpen)
        {
            // openGripper();
        }
        else
        {
            // closeGripper();
        }
        currentGripperState = GRIPPER_IDLE_MOVING_UR5;
        break;
    case STOP:
        // stop the robot
        if (programIsRunning)
        {
            Serial.print("Stopping the robot");
            digitalWrite(BOOT_PIN, LOW); // turn of the boot pin
            programIsRunning = false;
            // gripperServo.detach();
            // pinMode(servoPin, OUTPUT);
            // digitalWrite(servoPin, 0);
            // if (digitalRead(resetPin) == 1)
            // {
            //     initGripper();
            // }

            break;
        }
    }
}
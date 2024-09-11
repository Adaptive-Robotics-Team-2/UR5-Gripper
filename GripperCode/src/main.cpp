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
// #define BOOT_PIN 6 // 4 on UR5
#define TRIG_PIN 8 // 6 on UR5

// define the UR5 pins
#define ECHO_PIN 4          // 6 on UR5
#define ECHO_FINISHED_PIN 3 // 5 on UR5

#pragma endregion

#pragma region PressureSensorDefinitions

const int PRESSURE_PIN = A0; // Analog pin for pressure sensor

int currentPressureValue = 0; // Variable to store pressure sensor value

const int pressureDroppedThreshold = 0;   // Threshold value indicating an object is dropped
const int pressureGripThreshold = 500;    // Threshold value indicating an object is picked up
const int pressureMaximumThreshold = 800; // Threshold value indicating Gripper is being to agressive

bool gripperIsOpen = true; // Flag to indicate if an object is picked up

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

GripperState currentGripperState = MOVE_GRIPPERS;

#pragma endregion

bool programIsRunning = false;
bool interruptOccurred = false;
// function prototypes
bool sendPulse(int pin);
int readPulse(int pin);
void initGripper();
// void initPressureSensorInterrupt();
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
    // pinMode(BOOT_PIN, OUTPUT);
    pinMode(ECHO_FINISHED_PIN, INPUT);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(BOOT_BUTTON, INPUT_PULLUP);
}

void initGripper()
{
    Serial.println("Initializing Gripper...");
    gripperServo.attach(SERVO_PIN);

    // if (!gripperServo.attach(SERVO_PIN))
    // {
    //     Serial.println("Failed to attach servo, stopping the robot.");
    //     currentGripperState = STOP;
    //     return;
    // }

    // Move the servo from 0 to 50 degrees in steps of 5 degrees
    for (int position = 0; position <= 100; position += 5)
    {
        gripperServo.write(position);
        delay(10); // Wait for the servo to reach the position

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
    // delay(100);
    // digitalWrite(pin, LOW); // Set the pin low again
    // Wait for response on pin ECHO_PIN
    while (readPulse(ECHO_PIN) != HIGH)
    {
        // Check if the interrupt has occurred
        if (interruptOccurred)
        {
            // Reset the shared variable
            // interruptOccurred = false;

            // Transition to the STOP state
            currentGripperState = STOP;
            return false; // Indicate that the function was interrupted
        }
    }
    digitalWrite(pin, LOW); // Set the pin low again
    Serial.print("Pin ");
    Serial.print(pin);
    Serial.println(" set LOW");

    Serial.print("Response received on pin ");
    Serial.println(ECHO_PIN);

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
    while (currentPosition >= openPosition)
    {
        if (interruptOccurred)
        {
            break;
            ;
        }
        currentPosition -= 5;
        gripperServo.write(currentPosition);
        delay(50);
        currentPressureValue = analogRead(PRESSURE_PIN);
        // Serial.print("Pressure Value: ");
        // Serial.println(currentPressureValue);
        if (currentPressureValue >= pressureMaximumThreshold)
        {
            Serial.print("ERROR");
            Serial.println(currentPosition);
            closePosition = currentPosition; // Store the position where the object was gripped
            currentGripperState = STOP;
            break;
        }
    }

    gripperIsOpen = true;
    Serial.println("Gripper fully opened.");

    Serial.println("Gripper Open...");
}

void closeGripper()
{
    Serial.println("Closing Grippers...");
    Serial.println(currentPosition);
    while (currentPressureValue <= pressureGripThreshold && currentPosition < 100)
    { // Allow movement up to 120 degrees
        if (interruptOccurred)
        {
            break;
        }
        currentPosition += 10;
        gripperServo.write(currentPosition);
        delay(10);

        currentPressureValue = analogRead(PRESSURE_PIN);
        Serial.print("Pressure Value: ");
        Serial.println(currentPressureValue);

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

void interruptHandler()
{
    currentGripperState = STOP;
    interruptOccurred = true;
    Serial.println("UR5 has arrived at its destination");
}

void setup()
{
    Serial.begin(9600);

    initControlPins();
    initGripper();
    // initPressureSensorInterrupt();
    Serial.println("Setup complete...");

    attachInterrupt(digitalPinToInterrupt(ECHO_FINISHED_PIN), interruptHandler, RISING);
}

void loop()
{
    buttonState = digitalRead(BOOT_BUTTON);
    currentPressureValue = analogRead(PRESSURE_PIN);
    Serial.print("Pressure Value: ");
    Serial.println(currentPressureValue);

    if (buttonState == 99 && programIsRunning == false)
    {
        Serial.println("Booting up the robot again");
        currentGripperState = BOOT;
    }
    else if (buttonState == 99 && programIsRunning == true)
    {
        Serial.println("Stopping the robot");
        currentGripperState = STOP;
    }

    // when the robot is done with all its steps, it will stop and wait for the button to be pressed
    if (readPulse(ECHO_FINISHED_PIN) == HIGH)
    {
        currentGripperState = STOP;
    }

    Serial.println("Current gripper state: ");
    Serial.println(currentGripperState);
    switch (currentGripperState)
    {
    case BOOT:
        digitalWrite(TRIG_PIN, LOW);
        // send a pulse to the boot pin
        Serial.println("\nBooting up the robot");
        delay(10000);
        // UR5 should send messege when it is ready to use (at its starting position)
        if (sendPulse(TRIG_PIN))
        {
            programIsRunning = true;
            currentGripperState = MOVE_GRIPPERS;
            break;
        }
    case GRIPPER_IDLE_MOVING_UR5:
        Serial.println("\nGripper is idle");
        if (interruptOccurred)
        {
            break;
        }
        // Send pulse so the UR5 moves to its next location.
        // UR5 sends message back when it arrives
        if (sendPulse(TRIG_PIN))
        {
            Serial.println("UR5 moved to next location");

            // gripperIsOpen = !gripperIsOpen; // toggle the gripper currentGripperState when the arm is done moving
            currentGripperState = MOVE_GRIPPERS;
        }

    case MOVE_GRIPPERS:
        Serial.println("\nMoving grippers");
        // gripperIsOpen = true;
        if (interruptOccurred)
        {
            break;
        }

        // based on the gripper currentGripperState, open or close the gripper
        if (!gripperIsOpen)
        {
            Serial.println("Opening Grippers...");
            openGripper();
        }
        else
        {
            Serial.println("Closing Grippers...");
            closeGripper();
        }
        currentGripperState = GRIPPER_IDLE_MOVING_UR5;
        break;
    case STOP:
        // stop the robot
        if (programIsRunning)
        {
            Serial.println("\nStopping the robot");

            Serial.println("Waiting for the button to be pressed to rerstart the program");

            // digitalWrite(BOOT_PIN, LOW); // turn of the boot pin
            digitalWrite(TRIG_PIN, LOW); // turn of the trigger pin
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
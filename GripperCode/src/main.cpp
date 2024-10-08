#include <Arduino.h>
#include <Servo.h>
#include <TimerOne.h> // Include the TimerOne library

enum GripperState
{
    BOOT_UP,    // Initial state during boot-up
    STATE_IDLE, // Idle state when not moving
    MOVING,     // State when the gripper is moving
    PICK_UP,    // State when picking up an object
    DROP        // State when dropping an object
};

const int PRESSURE_PIN = A0; // Analog pin for pressure sensor
const int SERVO_PIN = 11;    // Digital pin for servo motor
const int TRIGGER_PIN = 2;   // Define the pin to trigger the idle state change

int pressureValue = 0;                     // Variable to store pressure sensor value
const int pressureDroppedThreshold = 0;    // Threshold value indicating an object is dropped
const int pressureThreshold = 300;         // Threshold value for pressure sensor
const int pressurePickedUpThreshold = 500; // Threshold value indicating something is picked up
bool pickedUp = false;                     // Flag to indicate if an object is picked up

Servo gripperServo;                  // Servo object for the gripper
GripperState currentState = BOOT_UP; // Initial state of the gripper

int closingPosition = 45; // Position to close the gripper
int openingPosition = 0;  // Position to open the gripper

bool flag = false;

// Function to be called by the timer interrupt
void timerISR()
{
    pressureValue = analogRead(analogPin); // Read the analog pin
    if (pressureValue > 900)
    {
        flag = true;
    }
}

void InitGripper()
{
    Serial.println("Initializing Gripper...");
    if (!gripperServo.attach(SERVO_PIN))
    {
        Serial.println("Failed to attach servo");
        return;
    }

    // Move the servo from 0 to 50 degrees in steps of 5 degrees
    for (int position = 0; position <= 50; position += 5)
    {
        gripperServo.write(position);
        delay(100); // Wait for the servo to reach the position

        // Read the pressure sensor value
        pressureValue = analogRead(PRESSURE_PIN);

        if (pressureValue > pressureThreshold)
        {
            Serial.print("Pressure exceeded threshold at position: ");
            Serial.println(position);
            currentState = STATE_IDLE;
            break;
        }
    }

    delay(1000); // Delay for initialization
}

void MoveToLocation()
{
    // Pin needs to be set so the UR5 can move to the target position
}

void CloseGrippers()
{
    Serial.println("Closing Grippers...");
    gripperServo.write(closingPosition); // Assuming 45 degrees closes the grippers
    if (pressureValue > pressurePickedUpThreshold)
    {
        pickedUp = true;
    }
}

void OpenGrippers()
{
    Serial.println("Opening Grippers...");
    gripperServo.write(openingPosition); // Assuming 0 degrees opens the grippers
    if (pressureValue < pressureDroppedThreshold)
    {
        pickedUp = false;
    }
}

void setup()
{
    pinMode(TRIGGER_PIN, INPUT); // Set the trigger pin as input
    Serial.begin(115200);
    InitGripper();
    // Initialize Timer1 to call timerISR every 50 milliseconds
    Timer1.initialize(50000);         // 50000 microseconds = 50 milliseconds
    Timer1.attachInterrupt(timerISR); // Attach the interrupt service routine
}

void loop()
{
    switch (currentState)
    {
    case BOOT_UP:
        if (pressureValue <= pressureThreshold)
        {
            currentState = STATE_IDLE;
            Serial.println("Transition to STATE_IDLE state.");
        }
        break;

    case STATE_IDLE:
        // Transition to MOVING state when the trigger pin is high
        if (digitalRead(TRIGGER_PIN) == HIGH)
        {
            currentState = MOVING;
            Serial.println("Transition to MOVING state.");
        }
        break;

    case MOVING:
        MoveToLocation(); // Example target position

        if (pressureValue > pressureThreshold)
        {
            if (pickedUp)
            {
                currentState = DROP;
                Serial.println("Transition to DROP state.");
            }
            else
            {
                currentState = PICK_UP;
                Serial.println("Transition to PICK_UP state.");
            }
        }
        break;

    case PICK_UP:
        CloseGrippers();
        currentState = STATE_IDLE; // Return to STATE_STATE_IDLE after pickup
        Serial.println("Returning to STATE_IDLE state after PICK_UP.");
        break;

    case DROP:
        OpenGrippers();
        currentState = STATE_IDLE; // Return to STATE_STATE_IDLE after drop
        Serial.println("Returning to STATE_IDLE state after DROP.");
        break;
    }

    delay(1000); // Delay between state checks
}
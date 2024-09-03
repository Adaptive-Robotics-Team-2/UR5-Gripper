#include <Servo.h>

enum GripperState {
    BOOT_UP,
    STATE_IDLE,
    MOVING,
    PICK_UP,
    DROP
};

const int PRESSURE_PIN = A0;
const int servoPin = 11;
const int pressureThreshold = 300;

Servo gripperServo;
GripperState currentState = BOOT_UP;
bool pickedUp = false;
int pressureValue = 0;

void InitGripper() {
    Serial.println("Initializing Gripper...");
    gripperServo.attach(servoPin);
       for (int position = 0; position <= 50; position += 5) {
        gripperServo.write(position); 
        delay(100);               

        pressureValue = analogRead(PRESSURE_PIN);

        if (pressureValue > pressureThreshold) {
          Serial.print("Pressure exceeded threshold at position: ");
          Serial.println(position);
          currentState = STATE_IDLE;
          break;
        }
      }
   
    delay(1000);  // Delay for initialization
}

void MoveToLocation(int targetPosition) {
    Serial.print("Moving to position: ");
    Serial.println(targetPosition);
    gripperServo.write(targetPosition);
    delay(1000);  // Delay for movement
}

void CloseGrippers() {
    Serial.println("Closing Grippers...");
    gripperServo.write(45);  // Assuming 45 degrees closes the grippers
    pickedUp = true;
}

void OpenGrippers() {
    Serial.println("Opening Grippers...");
    gripperServo.write(0);  // Assuming 0 degrees opens the grippers
    pickedUp = false;
}

void setup() {
    Serial.begin(115200);
    InitGripper();
}

void loop() {
    switch (currentState) {
        case BOOT_UP:
            pressureValue = analogRead(PRESSURE_PIN);
            if (pressureValue <= pressureThreshold) {
                currentState = STATE_IDLE;
                Serial.println("Transition to STATE_STATE_IDLE state.");
            }
            break;

        case STATE_IDLE:
            // Assume some condition or command triggers the move
            if (/* CommandStartProcess */ true) {  // Replace with actual condition
                currentState = MOVING;
                Serial.println("Transition to MOVING state.");
            }
            break;

        case MOVING:
            MoveToLocation(90);  // Example target position

            pressureValue = analogRead(PRESSURE_PIN);
            if (pressureValue > pressureThreshold) {
                if (pickedUp) {
                    currentState = DROP;
                    Serial.println("Transition to DROP state.");
                } else {
                    currentState = PICK_UP;
                    Serial.println("Transition to PICK_UP state.");
                }
            }
            break;

        case PICK_UP:
            CloseGrippers();
            currentState = STATE_IDLE;  // Return to STATE_STATE_IDLE after pickup
            Serial.println("Returning to STATE_IDLE state after PICK_UP.");
            break;

        case DROP:
            OpenGrippers();
            currentState = STATE_IDLE;  // Return to STATE_STATE_IDLE after drop
            Serial.println("Returning to STATE_IDLE state after DROP.");
            break;
    }

    delay(1000);  // Delay between state checks
}

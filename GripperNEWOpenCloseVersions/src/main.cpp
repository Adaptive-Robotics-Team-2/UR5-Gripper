#include <Arduino.h>
#include <Servo.h>

enum GripperState {
  BOOT_UP,
  STATE_IDLE,
  MOVING,
  PICK_UP,
  DROP,
  STOP
};

const int PRESSURE_PIN = A0;
const int resetPin = 1;
const int servoPin = 11;
const int pressureThreshold = 800;
const int pressureGrip = 500;
const int openPosition = 0;  // Gripper fully open
int currentPosition = 0;     // Current servo position, starts at 0
int closePosition = 0;       // Variable to store the position where the object is gripped

Servo gripperServo;
GripperState currentState = BOOT_UP;
bool pickedUp = false;
int pressureValue = 0;

void InitGripper() {
  Serial.println("Initializing Gripper...");
  gripperServo.attach(servoPin);
  gripperServo.write(0);
  pressureValue = analogRead(PRESSURE_PIN);
  if (pressureValue >= pressureGrip) {
    Serial.print("ERROR ");
    Serial.println(currentPosition);
    closePosition = currentPosition;  // Store the position where the object was gripped
    currentState = STOP;
  }
  currentState = STATE_IDLE;
}

//  for (int position = 0; position <= 50; position += 5)
//  {
//    gripperServo.write(position);
//    delay(50);
//    if (pressureValue >= pressureGrip) {
//      Serial.print("Object gripped at position: ");
//      Serial.println(currentPosition);
//      closePosition = currentPosition;  // Store the position where the object was gripped
//      currentState = STOP;
//      break;
//    }
//  }
//  delay(1000);  // Delay for initialization
//  for (int position = 50; position <= 0; position -= 5)
//  {
//    gripperServo.write(position);
//    delay(50);
//    if (pressureValue >= pressureGrip) {
//      Serial.print("Object gripped at position: ");
//      Serial.println(currentPosition);
//      closePosition = currentPosition;  // Store the position where the object was gripped
//      currentState = STOP;
//      break;
//    }
//  }
//  delay(1000);
//  currentState = BOOT_UP;
//}

void MoveToLocation(int targetPosition) {
  Serial.print("Moving to position: ");
  Serial.println(targetPosition);
  gripperServo.write(targetPosition);
  delay(1000);  // Delay for movement
  currentPosition = targetPosition;  // Update current position after movement
}

void CloseGrippers() {
  Serial.println("Closing Grippers...");
  while (pressureValue <= pressureGrip && currentPosition < 120) {  // Allow movement up to 120 degrees
    currentPosition += 10;
    gripperServo.write(currentPosition);
    delay(100);

    pressureValue = analogRead(PRESSURE_PIN);
    if (pressureValue >= pressureGrip) {
      Serial.print("Object gripped at position: ");
      Serial.println(currentPosition);
      closePosition = currentPosition;  // Store the position where the object was gripped
      currentState = STOP;
      break;
    }
  }
  pickedUp = true;  // Object has been picked up
}

void OpenGrippers() {
  if (pickedUp) {
    Serial.println("Opening Grippers to the position where object was gripped...");
    while (currentPosition > openPosition) {
      currentPosition -= 5;
      gripperServo.write(currentPosition);
      delay(50);
      pressureValue = analogRead(PRESSURE_PIN);
      if (pressureValue >= pressureGrip) {
        Serial.print("Object gripped at position: ");
        Serial.println(currentPosition);
        closePosition = currentPosition;  // Store the position where the object was gripped
        currentState = STOP;
        break;
      }

      if (currentPosition <= openPosition) {
        Serial.println("Gripper fully opened.");
        currentState = STATE_IDLE;
        pickedUp = false;
        break;
      }
    }
  }
  //  else {
  //    Serial.println("Opening Grippers...");
  //    for (int position = 50; position <= 0; position -= 5)
  //    {
  //      gripperServo.write(position);
  //      delay(50);
  //      pressureValue = analogRead(PRESSURE_PIN);
  //      if (pressureValue >= pressureGrip) {
  //        Serial.print("Object gripped at position: ");
  //        Serial.println(currentPosition);
  //        closePosition = currentPosition;  // Store the position where the object was gripped
  //        currentState = STOP;
  //        break;
  //      }
  //      if (position == 0) {
  //        Serial.println("Gripper fully opened.");
  //        currentState = STATE_IDLE;
  //        pickedUp = false;
  //        break;

}

void setup() {
  Serial.begin(115200);
  InitGripper();
}

void loop() {
  pressureValue = analogRead(PRESSURE_PIN);
  Serial.println(pressureValue);
  switch (currentState) {
    case BOOT_UP:
      currentState = STATE_IDLE;
      Serial.println("Transition to STATE_IDLE.");
      break;

    case STATE_IDLE:
      // Example of transition logic: can be triggered by a command or sensor input
      currentState = MOVING;
      Serial.println("Transition to MOVING state.");
      break;

    case MOVING:
    //      MoveToLocation(50);  // Example target position
    //      currentState = PICK_UP;
    //      Serial.println("Transition to PICK_UP state.");
    //      break;

    case PICK_UP:
      CloseGrippers();
      currentState = STATE_IDLE;  // Return to STATE_IDLE after pickup
      Serial.println("Returning to STATE_IDLE after PICK_UP.");
      break;

    case DROP:
      OpenGrippers();
      currentState = STATE_IDLE;  // Return to STATE_IDLE after drop
      Serial.println("Returning to STATE_IDLE after DROP.");
      break;

    case STOP:
      Serial.println("Gripper in STOP state.");
      gripperServo.detach();
      pinMode(servoPin, OUTPUT);
      digitalWrite(servoPin, 0);
      if (digitalRead(resetPin) == 1) {
        InitGripper();
      }
      break;
  }
}

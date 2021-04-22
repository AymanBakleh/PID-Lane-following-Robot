
#include <AccelStepper.h>//you need the library

AccelStepper LeftBackWheel(1, 2, 5);   // x(Type:driver, STEP, DIR) - Stepper1
AccelStepper LeftFrontWheel(1, 3, 6);  //y Stepper2
AccelStepper RightBackWheel(1, 4, 7);  // zStepper3
AccelStepper RightFrontWheel(1, 12, 13); //a Stepper4
int wheelSpeed = 400;
char order;
void setup() {
  // put your setup code here, to run once:
  LeftFrontWheel.setMaxSpeed(wheelSpeed);
  LeftFrontWheel.setAcceleration(1000);
  LeftBackWheel.setMaxSpeed(wheelSpeed);
  LeftBackWheel.setAcceleration(1000);
  RightFrontWheel.setMaxSpeed(wheelSpeed);
  RightFrontWheel.setAcceleration(1000);
  RightBackWheel.setMaxSpeed(wheelSpeed);
  RightBackWheel.setAcceleration(1000);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    order = Serial.read();
    if (order == 'f')
    { moveForward();
    Serial.println("Forward") ;
    }
    if (order == 'b')
    { moveBackward() ;
    Serial.println("Backward") ;
    }
    if (order == 'r')
    { moveSidewaysRight();
    Serial.println("side ways Right") ;
    }
    if (order == 'l')
    { moveSidewaysLeft();
    Serial.println("Side ways Left") ;
    }
    if (order == 'L')
    { rotateLeft();
    Serial.println("Rotate Left") ;
    }
    if (order == 'R')
    { rotateRight();
    Serial.println("Rotate Right") ;
    }
    if (order == 'F')
    { moveRightForward() ;
    Serial.println("Forward Right") ;
    }
    if (order == 'B')
    { moveRightBackward();
    Serial.println("Backward Right") ;
    }
    if (order == 'E')
    { moveLeftForward();
    Serial.println("Forward Left") ;
    }
    if (order == 'P')
    { moveLeftBackward();
    Serial.println("Backward Left") ;
    }
    if (order == 's')
    { stopMoving();
    Serial.println("Stop!") ;
    }
  }
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void moveForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void moveBackward() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void moveSidewaysRight() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void moveSidewaysLeft() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void rotateLeft() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void rotateRight() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void moveRightForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(wheelSpeed);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void moveRightBackward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(0);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void moveLeftForward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(0);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void moveLeftBackward() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(-wheelSpeed);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void stopMoving() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(0);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
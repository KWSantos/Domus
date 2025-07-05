#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <ESP32Servo.h>

#define CLAW_SERVO_PIN 18
#define ARM_SERVO_PIN  19

Servo clawServo;
Servo armServo;

#define CLAW_OPEN_ANGLE   30
#define CLAW_CLOSED_ANGLE 0

#define ARM_UP_ANGLE   90
#define ARM_DOWN_ANGLE 0

/**
 * @brief Initializes and attaches the servos to their pins.
 * Sets a safe starting position for the arm and claw.
 */
void setupServos() {
  clawServo.attach(CLAW_SERVO_PIN);
  armServo.attach(ARM_SERVO_PIN);

  armServo.write(ARM_DOWN_ANGLE);
  delay(500);
  clawServo.write(CLAW_CLOSED_ANGLE);
  Serial.println("Servos configurados na posição inicial.");
}

/**
 * @brief Opens the claw.
 */
void openClaw() {
  clawServo.write(CLAW_OPEN_ANGLE);
}

/**
 * @brief Closes the claw.
 */
void closeClaw() {
  clawServo.write(CLAW_CLOSED_ANGLE);
}

/**
 * @brief Raises the arm.
 */
void raiseArm() {
  armServo.write(ARM_UP_ANGLE);
}

/**
 * @brief Lowers the arm.
 */
void lowerArm() {
  armServo.write(ARM_DOWN_ANGLE);
}

#endif 
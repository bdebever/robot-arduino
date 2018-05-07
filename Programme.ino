/**
 * Projet Arduino IE4 - asservissement d'un robot
 * @file    Programme.ino
 * @author  Baptiste Debever - Alexandre Deledalle
 * @version V0.0.1
 * @date    2018/05/01
 *
 * Function List:
 *    1. void MeMegaPiDCMotorTest::run(int16_t speed)
 *    2. void MeMegaPiDCMotorTest::stop(void)
 */

#include "MeEncoderMotor.h"
#include "MeMegaPi.h"
#include "Pixy.h"

MeEncoderMotor motorLeft(PORT1A);

MeEncoderMotor motorRight(PORT1B);

MeEncoderMotor motorFront(PORT2A);

uint8_t motorSpeed = 100;
int motorAngle = 10;

MeUltrasonicSensor ultraSensor(PORT_8); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */

/**
 * Fonction d'initialisation
 */
void setup()
{
  Serial.begin(9600);
}

/**
 * Fonction principale de loop du programme
 */
void loop()
{
  ultrasonStart();
  engineStart();
}

/**
 * Fonction permettant de mettre en marche notre capteur ultrason
 */
void ultrasonStart()
{
  Serial.print("Distance : ");
  Serial.print(ultraSensor.distanceCm() );
  Serial.println(" cm");
  delay(100); /* the minimal measure interval is 100 milliseconds */
}

/**
 * Fonction permettant de mettre en marche un moteur
 */
void engineStart()
{
  motorLeft.moveTo(motorAngle, motorSpeed); /* value: between -255 and 255. */
  motorRight.moveTo(motorAngle, motorSpeed); /* value: between -255 and 255. */
  motorFront.moveTo(motorAngle, motorSpeed);

  //float positionLeft = motorLeft.getCurrentPosition();
  //print(positionLeft);

  delay(100);
  motorLeft.moveTo(motorAngle, -motorSpeed);
  motorRight.moveTo(motorAngle, -motorSpeed);
  motorFront.moveTo(motorAngle, -motorSpeed);
  delay(2000);
}

/**
 * Projet Arduino IE4 - asservissement d'un robot
 * @file    Programme.ino
 * @author  Baptiste Debever - Alexandre Deledalle
 * @version V0.0.1
 * @date    2018/05/01
 * @link http://learn.makeblock.com/en/electronics/m
 *
 * Function List:
 *    1. void MeMegaPiDCMotorTest::run(int16_t speed)
 *    2. void MeMegaPiDCMotorTest::stop(void)
 */

#include "MeMegaPi.h"
#include "Pixy.h"

// Motors setup


/* Ultrasonic setup */
MeUltrasonicSensor ultraSensor(PORT_8);

// Pixy setup
Pixy pixy;

/**
 * Fonction d'initialisation
 */
void setup()
{
  Serial.begin(9600);
  pixy.init();
}

/**
 * Fonction principale de loop du programme
 */
void loop()
{
  getUltrasensorPosition();
  //engineStart();
}


/**
 * Fonction permettant de mettre en marche un moteur - pilotage
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

/**
 * Fonction permettant de get la vitesse du moteur
 */
float getMotorSpeed()
{

}

/**
 * Fonction permettant de get la position du moteur
 */
float getMotorPosition()
{

}

/**
 * Fonction permettant de tracer une courbe (eg: position, vitesse en fonction du temps)
 */
float draw()
{

}

/**
 * Fonction permettant de tracer une courbe (eg: position, vitesse en fonction du temps)
 */
int getUltrasensorPosition()
{
  Serial.print("Distance : ");
  Serial.print(ultraSensor.distanceCm() );
  Serial.println(" cm");
  delay(100);
}
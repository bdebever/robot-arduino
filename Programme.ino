/**
 * Projet Arduino IE4 - asservissement d'un robot
 * @file    Programme.ino
 * @author  Baptiste Debever - Alexandre Deledalle
 * @version V0.0.1
 * @date    2018/05/01
 * @link http://learn.makeblock.com/en/electronics/m
 *
 */

#include "MeMegaPi.h"
#include "Pixy.h"
#include "Arduino.h"

// Motors setup
int16_t moveSpeed = 180;
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard Encoder_3(SLOT3);

/* Ultrasonic setup */
MeUltrasonicSensor *us = NULL;

// Pixy setup
Pixy pixy;

/**
 * Fonction d'initialisation
 */
void setup()
{
  Serial.begin(9600);
  pixy.init();

  // Init Ultrason
  us = new MeUltrasonicSensor(PORT_7);
}

/**
 * Fonction principale de loop du programme
 */
void loop()
{
  int objectDetected = getUltrasensorValue();
  if (objectDetected <= 20) {
    // Move right
    Serial.print('Objet inférieure à 20cm détectée à: ' + positionDetected + 'cm');
  }
}


/**
 * Fonction permettant de mettre en marche un moteur - pilotage
 *
 * @var motor - le moteur à démarrer
 * @var movement - le mouvement souhaité
 */
void engineStart(int motor, int movement)
{

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
 * Fonction permettant de récupérer une valeur du capteur ultrason
 */
int getUltrasensorValue()
{
  return (float) us->distanceCm();
}

int getPixyValue()
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];

  // grab blocks!
  blocks = pixy.getBlocks();

  // If there are detect blocks, print them!
  if (blocks)
  {
    i++;

    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf);
        pixy.blocks[j].print();
      }
    }
  }
}


////**************** Fonctions supplementaires pour commande moteur ***************//////
/**
 * \par Function
 *    Forward
 * \par Description
 *    This function use to control the car kit go forward.
 */
void Forward(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

/**
 * \par Function
 *    Backward
 * \par Description
 *    This function use to control the car kit go backward.
 */
void Backward(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

/**
 * \par Function
 *    BackwardAndTurnLeft
 * \par Description
 *    This function use to control the car kit go backward and turn left.
 */
void BackwardAndTurnLeft(void)
{
  Encoder_1.setMotorPwm(-moveSpeed/4);
  Encoder_2.setMotorPwm(moveSpeed);
}

/**
 * \par Function
 *    BackwardAndTurnRight
 * \par Description
 *    This function use to control the car kit go backward and turn right.
 */
void BackwardAndTurnRight(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed/4);
}

/**
 * \par Function
 *    TurnLeft
 * \par Description
 *    This function use to control the car kit go backward and turn left.
 */
void TurnLeft(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed/2);
}

/**
 * \par Function
 *    TurnRight
 * \par Description
 *    This function use to control the car kit go backward and turn right.
 */
void TurnRight(void)
{
  Encoder_1.setMotorPwm(moveSpeed/2);
  Encoder_2.setMotorPwm(-moveSpeed);
}

/**
 * \par Function
 *    TurnLeft1
 * \par Description
 *    This function use to control the car kit go backward and turn left(fast).
 */
void TurnLeft1(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

/**
 * \par Function
 *    TurnRight1
 * \par Description
 *    This function use to control the car kit go backward and turn right(fast).
 */
void TurnRight1(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

/**
 * \par Function
 *    Stop
 * \par Description
 *    This function use to stop the car kit.
 */
void Stop(void)
{
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}

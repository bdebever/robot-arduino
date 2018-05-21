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
#include "SimpleTimer.h"

SimpleTimer timer;                 // Timer pour échantillonnage
unsigned int tick_codeuse = 0;     // Compteur de tick de la codeuse

boolean entryFound = false;
boolean exitFound = false;
int distance = 0;
int axeCentre = 0; // Decalage par rapport au centre

// la liste des états possible de notre système
enum {FORWARD, SIDEWARD, STANDSTILL} direction;

unsigned long temps_actuel;
unsigned long mem_temps;

// Distance parcourue en 1 tour moteur
const float distanceMotorLapse = 26.8;

// Motors setup
int16_t moveSpeed = 50;
MeEncoderOnBoard Encoder_1(PORT_1);
MeEncoderOnBoard Encoder_2(PORT_2);
MeEncoderOnBoard Encoder_3(PORT_3);

/* Ultrasonic setup */
MeUltrasonicSensor *us = NULL;     //PORT_7

// Pixy setup
Pixy pixy;

/// Interruption pour encodeur1
void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

/// Interrtuption pour encodeur2
void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

/// Interrtuption pour encodeur3
void isr_process_encoder3(void)
{
  if(digitalRead(Encoder_3.getPortB()) == 0)
  {
    Encoder_3.pulsePosMinus();
  }
  else
  {
    Encoder_3.pulsePosPlus();
  }
}

/**
 * Fonction d'initialisation
 */
void setup()
{
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(Encoder_3.getIntNum(), isr_process_encoder3, RISING);
  Serial.begin(115200);  // Init communication série (USB-série)

  //Set PWM à 1khz
  TCCR1A = _BV(WGM10);//PIN12
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);//PIN8
  TCCR2B = _BV(CS22);

  TCCR3A = _BV(WGM30);//PIN9
  TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);

  TCCR4A = _BV(WGM40);//PIN5
  TCCR4B = _BV(CS41) | _BV(CS40) | _BV(WGM42);
  // Fin Set PWM
  pixy.init();

  us = new MeUltrasonicSensor(PORT_7); // Init Ultrason
}

/**
 * Fonction principale de loop du programme
 */
void loop()
{
  // Variables locales
  float speed1, speed2, speed3;

  //--- Attente ---//
  // Réalisation d'une attente avec la resynchronisation du programme
  do
  {
    temps_actuel = millis() ;
  }
  while((temps_actuel-mem_temps)<=temps_attente);  //
  mem_temps = temps_actuel;

  // Lecture encodeurs
  Encoder_1.loop();
  Encoder_2.loop();
  Encoder_3.loop();
  speed1 = Encoder_1.getCurrentSpeed();
  speed2 = Encoder_2.getCurrentSpeed();
  speed3 = Encoder_3.getCurrentSpeed();

  // On interroge l'ultrason
  getUltrasensorValue();

  // On avance
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_3.setMotorPwm(-moveSpeed);
  direction = FORWARD;

  //--- Machine d'etat - similaire a un Grafcet ---//
  // Comme un Grafcet

  switch(direction)
  {
      case FORWARD:
        // Distance superieure a 20 - aucun besoin de s'inquieter
        if (distance > 20)
          break;

        // On arrete le moteur car distance inferieure a 20
        stopMotor();
        direction = STANDSTILL;
        break;

      case SIDEWARD:

        // Rien en vue -> on peut se decaler vers la droite
        if (distance > 20) {
          forward(true);
          axeCentre += 1;
          // On se replace vers lavant
          turnLeft(90);
        } else {
          turnLeft(180);
          // On suppose donc quil ny a rien
          forward(true);
          axeCentre -= 1;
          turnRight(90);
        }

        // On repart de lavant
        forward(false);
        // Set le nouvel etat
        direction = FORWARD;
        break;

      case STANDSTILL:

        // Demander a Pixy si on detecte un object connu
        if (getPixyValue()) {

          // Sil sagit de la porte dentree (on suppose rien dautre pour linstant)
          // se decaler

          // Sil sagit de la porte de sortie

          // Stop
        } else {
          // Sinon, on regarde sur la droite
          turnRight(90);
          getUltrasensorValue();
          delay(100);

        }

        // Set le nouvel etat
        direction = SIDEWARD;
        break;
  }


  //--- Ecritures des valeurs (Print) ---//

  Serial.print("Distance:");
  Serial.print(distance);
  Serial.print("Megapi:");
  Serial.print(distance);

  //--> Ecriture pour le traceur série
  Serial.print(speed1);
  Serial.print(", ");
  Serial.print(speed2);
  Serial.print(", ");
  Serial.print(speed3);

  // We don't have the ball: get the distance with the Pixy
  if (!entryFound) {
    Serial.print("ball: ");
    Serial.println(entryFound);


    Serial.print(Encoder_1.getCurrentSpeed());
    Serial.print(Encoder_3.getCurrentSpeed());

    if (getUltrasensorValue() < 5) {
        entryFound = true;
        Serial.println(Encoder_1.getCurrentSpeed());
    }


  }


}


/**
 * Fonction permettant de se tourner vers la droite
 *
 * @var motor - la rotation
 */
void turnRight(int rotation)
{
  Encoder_1.setMotorPwm(-rotation);
  Encoder_3.setMotorPwm(+rotation);
  delay(500);
  Encoder_1.setMotorPwm(0);
  Encoder_3.setMotorPwm(0);
}

/**
 * Fonction permettant de se tourner vers la gauche
 *
 * @var motor - la rotation
 */
void turnLeft(int rotation)
{
  Encoder_1.setMotorPwm(+rotation);
  Encoder_3.setMotorPwm(-rotation);
  delay(500);
  Encoder_1.setMotorPwm(0);
  Encoder_3.setMotorPwm(0);
}

/**
 * Avancer d'une vitesse
 */
void forward()
{
  // Distance to go in degrees (360 = 1 lapse)
  //int distanceToGo = ((distanceGo * 360) / distanceMotorLapse);
  Encoder_1.setMotorPwm(-80*1.2);
  Encoder_3.setMotorPwm(-80);
  delay(600);
  Encoder_1.setMotorPwm(0);
  Encoder_3.setMotorPwm(0);
}

void stopMotor()
{
  Encoder_1.setMotorPwm(0);
  Encoder_3.setMotorPwm(0);
}

/**
 * Fonction permettant de get la vitesse du moteur
 */
void getMotorSpeed()
{
  Serial.print("Moteur 1: ");
  Serial.println(Encoder_1.getCurrentSpeed());
  Serial.print("Moteur 2: ");
  Serial.println(Encoder_2.getCurrentSpeed());
  Serial.print("Moteur 3: ");
  Serial.println(Encoder_3.getCurrentSpeed());
}

/**
 * Fonction permettant de tracer une courbe (eg: position, vitesse en fonction du temps)
 */
void draw()
{
  // Lecture des encodeurs
  Encoder_1.loop();
  Encoder_2.loop();
  Encoder_3.loop();
  Serial.print(Encoder_1.getCurrentSpeed());
  delay(10);
  Serial.println();
}

/**
 * Fonction permettant de rÃ©cupÃ©rer une valeur du capteur ultrason
 */
float getUltrasensorValue()
{
  distance = (int) us->distanceCm();
  return distance;
}

/**
 *
 */
void getPixyValue()
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


////**************** Fonctions relatives au PID ***************//////
/* Interruption sur tick de la codeuse */
void compteur(){
    tick_codeuse++;  // On incrémente le nombre de tick de la codeuse
}


/**
 * Interruption pour calcul du PID
 * @link http://www.ferdinandpiette.com/blog/2012/04/asservissement-en-vitesse-dun-moteur-avec-arduino/
 */
void asservissement(float consigne, int moteur)
{
    // DEBUG
    Serial.println(tick_codeuse);

    // Réinitialisation du nombre de tick de la codeuse
    tick_codeuse=0;

    //--- PID ---//
    erreur = consigne - valeur_mesuree;

    result_P = erreur * Kp;
    somme_erreurs +=  erreur;
    result_I = somme_erreurs * Ki;
    delta_erreur = erreur - erreur_precedente;
    erreur_precedente = erreur;
    result_D = delta_erreurs * Kd;

    // Notre commande
    commande = result_P + result_I + result_D;
    // saturation
    commande = constrain(commande,-255,255);

    // Reguler
    switch(moteur)
    {
        case 1:
          Encoder_1.setMotorPwm(commande);
          break;
        case 2:
          Encoder_2.setMotorPwm(commande);
        case 3:
          Encoder_3.setMotorPwm(commande);
    }
}

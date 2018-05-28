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

unsigned int    tickCodeuse = 0;     // Compteur de tick de la codeuse
boolean         entryFound = false;
boolean         exitFound = false;
int             distance = 0;
int             axeCentre = 0; // Decalage par rapport au centre
unsigned long   temps_attente;
float           speed1, speed2, speed3;
bool            debug = false;
bool            graph = true;
float           erreur;
float           somme_erreurs;
float           erreur_precedente;
unsigned long   valeur_mesuree;
int16_t         moveSpeed = 10;
const int       frequenceEchantillonnage = 50;
int             consigne = 10;
float           commande = 0;

int             pixySignature;
int             pixyX = 0;

// la liste des états possible de notre système
enum {FORWARD, SIDEWARD, STANDSTILL} direction;

// Motors setup
MeEncoderOnBoard Encoder_1(PORT_1);
MeEncoderOnBoard Encoder_2(PORT_2);
MeEncoderOnBoard Encoder_3(PORT_3);

// Capteur ultrasonor
MeUltrasonicSensor *us = NULL;     //PORT_7

// Pixy setup
Pixy pixy;
SimpleTimer timer;

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

  attachInterrupt(0, compteur, CHANGE);

  // Mise en place de l'asservissement
  timer.setInterval(1000 / frequenceEchantillonnage, asservissement);

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
  
  // Pixy et Ultrason setup
  pixy.init();
  us = new MeUltrasonicSensor(PORT_7); // Init Ultrason

  direction = FORWARD;
}

/**
 * Fonction principale de loop du programme
 */
void loop()
{
  // On lance le timer
  timer.run();

  // Lecture encodeurs
  Encoder_1.loop();
  Encoder_2.loop();
  Encoder_3.loop();
  speed1 = Encoder_1.getCurrentSpeed();
  speed2 = Encoder_2.getCurrentSpeed();
  speed3 = Encoder_3.getCurrentSpeed();

  // Demarrer moteurs
  //Encoder_1.moveTo(50);
  //Encoder_3.moveTo(50);
  
  // Debugs
  if (debug) {
    Serial.print("Distance : ");
    Serial.print(us->distanceCm());
    Serial.println(" cm");
    Serial.print("Position moteur 1 : ");
    Serial.println(Encoder_1.getCurPos());
    Serial.print("PixyX :");
    Serial.println(pixyX);
  }

  /**
   * On trace le graph
   */
  if (graph) {
    Serial.print(consigne);
    Serial.print(", ");
    Serial.print(Encoder_1.getCurrentSpeed());
    Serial.print(", ");
    Serial.println();
    delay(10);
  }

  // On interroge l'ultrason
  getUltrasensorValue();

}


/**
 * Arreter les moteurs
 */
void stopMotor(void)
{
  Encoder_1.setMotorPwm(0);
  Encoder_3.setMotorPwm(0);
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
 * Récuperer valeurs pixy
 */
void getPixyValue()
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];

  // grab blocks!
  blocks = pixy.getBlocks();

  if (blocks)
  {
    i++;

    // Chaque 5 frames
    if (i % 5 == 0)
    {

      if (debug) {
        sprintf(buf, "Detected %d:\n", blocks);
        Serial.print(buf);
      }

      // Pixel resolution --> 320 * 200, donc center à 160
      int minX = 160;

      for (j = 0; j < blocks; j++)
      {

        if (debug) {
          sprintf(buf, "  block %d: ", j);
          Serial.print(buf);
          pixy.blocks[j].print();
          Serial.print("\n");
        }

        pixySignature = pixy.blocks[j].signature;
        pixyX = pixy.blocks[j].x;
      }

    }
  }
   
}


/**
   \par Function
      Forward
   \par Description
      This function use to control the car kit go forward.
*/
void Forward(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_3.setMotorPwm(moveSpeed);
}

/**
   \par Function
      Backward
   \par Description
      This function use to control the car kit go backward.
*/
void Backward(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_3.setMotorPwm(-moveSpeed);
}

/**
   \par Function
      TurnLeft
   \par Description
      This function use to control the car kit go backward and turn left.
*/
void TurnLeft(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_3.setMotorPwm(moveSpeed / 2);
}

/**
   \par Function
      TurnLeft
   \par Description
      This function use to control the car kit go backward and turn left.
*/
void TurnRight(void)
{
  Encoder_1.setMotorPwm(moveSpeed / 2);
  Encoder_3.setMotorPwm(moveSpeed);
}

/**
   \par Function
      ReverseRight
   \par Description
      This function use to control the car kit go backward and turn right.
*/
void ReverseRight(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_3.setMotorPwm(moveSpeed);
}

/**
   \par Function
      TurnRight
   \par Description
      This function use to control the car kit go backward and turn right.
*/
void ReverseLeft(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_3.setMotorPwm(-moveSpeed);
}

/*
   Compteur pour le PID et l'asservisement
*/
void compteur() {
  tickCodeuse++;
}

/**
 * Interruption pour calcul du PID
 */
void asservissement()
{
    float result_P;
    float result_D;
    float result_I;
    float Kp = 0.66;
    float Ki = 0.7;
    float Kd = 0;
    float delta_erreurs;
    int minX = 160;

   int moteur = 1;

    switch(moteur)
    {
        case 1:
          valeur_mesuree = speed1;
          break;
        case 2:
          valeur_mesuree = speed2;
        case 3:
          valeur_mesuree = speed3;
          break;
    }

    //--- PID ---//
    erreur = consigne - Encoder_1.getCurrentSpeed();;
    
    result_P = erreur * Kp;
    somme_erreurs +=  erreur;
    
    result_I = somme_erreurs * Ki;
    delta_erreurs = erreur - erreur_precedente;
    
    erreur_precedente = erreur;
    result_D = delta_erreurs * Kd;

    // Notre commande
    commande = result_P + result_I + result_D;
       
    // On assigne la valeur aux moteurs
    moveSpeed = commande;

    // Appeler Pixy 
    getPixyValue();

  
  // Si présence drapeau: arret
    if (pixySignature == 1 && 
      minX - 50 <= pixyX <= minX + 50) {
      // Carton rouge: stop
      stopMotor();
      while(1){};// Stop
    }

    while(pixySignature == 2) {
      if(pixyX < 160) {
        TurnRight();
      } else {
        TurnLeft();
      }
  
      getUltrasensorValue();
  
      // Eviter obstacle intermédiaire
      if (distance < 30)
        break;
      
      // Refresh;
      getPixyValue();
    }

    if (distance > 30) {
      Forward();
      // On a un objet (joueur)  
    } else {
      // Sinon, on tourne sur la droite
      if (debug)
        Serial.println("Obstacle detecte");
      
      Backward();
      delay(500);
      //Forward();
      delay(2000);
      TurnLeft();
      delay(200);
      
    }
  
  /*
  switch(direction)
  {
      case FORWARD:
        // Distance superieure a 20 - aucun besoin de s'inquieter
        if (distance > 20) {
          Forward();
          break;
        }

        // On arrete le moteur car distance inferieure a 20
        stopMotor();
        direction = STANDSTILL;
        break;

      case SIDEWARD:

        // Rien en vue -> on peut se decaler vers la droite
        if (distance > 20) {
          Forward();
          delay(200);
          stopMotor();
          // On se replace vers lavant
          TurnLeft();
        } else {
          TurnLeft();
          // On suppose donc quil ny a rien
          Forward();
          axeCentre -= 1;
          TurnRight();
        }

        // Set le nouvel etat
        direction = FORWARD;
        break;

      case STANDSTILL:

        // Demander a Pixy si on detecte un object connu
        if (getPixyValue()) {
          // Sil sagit de la porte de sortie: stop
          stopMotor();
          while(1){};
          // Stop
        } else {
          // Sinon, on regarde sur la droite
          TurnRight();
          delay(100);
        //}

        // Set le nouvel etat
        direction = SIDEWARD;
        break;
  }*/
}


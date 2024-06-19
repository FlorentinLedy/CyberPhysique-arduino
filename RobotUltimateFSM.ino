#include "MeMegaPi.h"
#include <Wire.h>
#include <TimerFive.h>

// Définitions de constantes pour le robot
#define BELT_PITCH 2.032 // distance entre les dents de la courroie (en mm)
#define NTEETH 90.0 // nombre de dents sur la courroie

// Conversion de la vitesse de tr/min en mm/s
#define RPM_2_MMS BELT_PITCH * NTEETH / 60.0 

// Conversion de la tension en PWM
#define VOLTS_2_PWM 255.0 / 12.0 
#define MAX_VOLTAGE 9 // tension maximale autorisée (en V)

#define Ts 5000 // Taux d'échantillonnage en µs

// Constantes pour le contrôle
#define Tau 0.02089f
#define k 54.44f
float a = -1 / Tau;
float b = k / Tau;

#define l1 382.96f
#define l2 41257.0f
#define l3 1010.2f
#define kI -1.3190f
#define kx 0.0276f
#define Kb 52.0f

// Variables pour le contrôle
float e1 = 0;
float e2 = 0;
float xi01 = 0;
float xi02 = 0;
float p01 = 0;
float p02 = 0;
float v01 = 0;
float v02 = 0;
float d01 = 0;
float d02 = 0;
float Te = (Ts / 1000) / 1000.0f; // période d'échantillonnage (en s)

// Variables de l'état du robot
volatile float angle = 0; // cap actuel (en degrés)
volatile float angle_point = 0; // taux de variation de l'angle (en degrés/s)
volatile float speed1 = 0; // vitesse actuelle du moteur 1 (en mm/s)
volatile float speed2 = 0; // vitesse actuelle du moteur 2 (en mm/s)
volatile float position1 = 0; // position actuelle du moteur 1 (en mm)
volatile float position2 = 0; // position actuelle du moteur 2 (en mm)
float u1 = 0; // signal de contrôle pour le moteur 1 (en V)
float u2 = 0; // signal de contrôle pour le moteur 2 (en V)
volatile long compTime = 0; // temps de calcul de la boucle critique (en µs)
volatile short overrun = 0; // indicateur de dépassement
volatile float ref1 = 0; // référence de vitesse pour le moteur 1
volatile float ref2 = 0; // référence de vitesse pour le moteur 2
volatile float refAngle = 0; // référence de cap (en degrés)

// Cible à atteindre (en mm)
const float targetX = 2000; // distance cible sur l'axe X (en mm)
const float targetY = 2000; // distance cible sur l'axe Y (en mm)

// Vitesse de référence (en mm/s)
float vxref = 0;
float vyref = 0;

// Position actuelle (en mm)
float x = 0;
float y = 0;

// Vitesse précédente
float vxref_pres = 0;
float vyref_pres = 0;

// Instanciation des objets gyroscope et encodeurs
MeGyro gyro; // instanciation de l'objet gyroscope
MeEncoderOnBoard Encoder_1(SLOT1); // instanciation de l'objet encodeur pour le moteur 1
MeEncoderOnBoard Encoder_2(SLOT2); // instanciation de l'objet encodeur pour le moteur 2

// Routine appelée toutes les 5ms par la routine ISR
void Update5ms()
{
    UpdateSensors();
    UpdateControl();
    UpdateActuators();
}

// Fonction pour limiter la valeur du signal de contrôle
float calcU(float u) {
    if (u > 9.0f)
        return 9.0f;
    else if (u < -9.0f)
        return -9.0f;
    return u;
}

// Mise à jour des capteurs
void UpdateSensors(){
    float angle_anc = angle;
    gyro.update(); // mise à jour de l'état du gyroscope
    angle = gyro.getAngleZ(); // obtention du cap estimé (en degrés)
    angle_point = (angle - angle_anc) / Te; // calcul du taux de variation de l'angle
    Encoder_1.loop(); // mise à jour de l'état de l'encodeur 1
    Encoder_2.loop(); // mise à jour de l'état de l'encodeur 2

    speed1 = Encoder_1.getCurrentSpeed() * RPM_2_MMS; // calcul de la vitesse du moteur 1 (en mm/s)
    speed2 = Encoder_2.getCurrentSpeed() * RPM_2_MMS; // calcul de la vitesse du moteur 2 (en mm/s)

    position1 = Encoder_1.getCurPos() * BELT_PITCH * NTEETH / 360.0f; // calcul de la position du moteur 1 (en mm)
    position2 = Encoder_2.getCurPos() * BELT_PITCH * NTEETH / 360.0f; // calcul de la position du moteur 2 (en mm)
}

// Mise à jour du contrôle
void UpdateControl()
{
    // Mettre à jour les vitesses de référence pour atteindre la cible
    vxref_pres = vxref;
    vyref_pres = vyref;

    // Calcul des nouvelles vitesses de référence
    vxref = 0.0333 * (targetX - x);
    vyref = 0.0333 * (targetY - y);

    // Mise à jour des positions actuelles
    x += (vxref + vxref_pres) / 2 * Te;
    y += (vyref + vyref_pres) / 2 * Te;

    float l = 175 / 2 * 85 / 100;
    float L = 124.5f;
    float refSpeedChenille = cos(angle * PI / 180) * vxref + sin(angle * PI / 180) * vyref;
    refAngle = (-sin(angle * PI / 180) / L) * vxref + (cos(angle * PI / 180) / L) * vyref;

    // Calcul de la différence de vitesse pour les chenilles
    float refSpeedDifference = (refAngle) * L;

    // Mise à jour des références de vitesse pour les moteurs
    ref1 = refSpeedChenille + refSpeedDifference;
    ref2 = refSpeedChenille - refSpeedDifference;

    // Mise à jour de la machine d'état pour le moteur 1
    e1 = position1 - d01;
    float d01temp = d01 + Te * (v01 + e1 * l1);
    float v01temp = v01 + Te * (a * v01 + b * u1 + b * p01 + l2 * e1);
    float p01temp = p01 + Te * l3 * e1;

    d01 = d01temp;
    v01 = v01temp;
    p01 = p01temp;

    // Mise à jour de la machine d'état pour le moteur 2
    e2 = position2 - d02;
    float d02temp = d02 + Te * (v02 + e2 * l1);
    float v02temp = v02 + Te * (a * v02 + b * u2 + b * p02 + l2 * e2);
    float p02temp = p02 + Te * l3 * e2;

    d02 = d02temp;
    v02 = v02temp;
    p02 = p02temp;

    // Calcul des signaux de contrôle
    float xi01temp = xi01 + Te * (-ref1 - v01 + Kb * (calcU(u1) - u1));
    float u10 = -kI * xi01 - kx * v01;

    float xi02temp = xi02 + Te * (-ref2 - v02 + Kb * (calcU(u2) - u2));
    float u20 = -kI * xi02 - kx * v02;

    xi01 = xi01temp;
    xi02 = xi02temp;
    u1 = u10;
    u2 = u20;
}

// Mise à jour des actionneurs
void UpdateActuators(){
    setMotorsVoltage(u1, u2); // réglage des tensions des moteurs
}

/*

routine appelée toutes les 5ms par le timer 5
*/
void Timer5ISR(){
    static char executing = 0; // indicateur d'exécution
    if(executing) {
        overrun = 1;
        return;
    }
    else executing = 1; // si déjà en cours d'exécution => dépassement

    interrupts(); // activer les interruptions pendant l'exécution de cette boucle
    long startTime = micros();

    Update5ms();

    compTime = micros() - startTime;
    executing = 0;
}

/*

routine d'interruption pour le comptage des impulsions de l'encodeur (quadrature)
*/
void isr_process_encoder1(void)
{
    if(digitalRead(Encoder_1.getPortB()) == 0)
    {
        Encoder_1.pulsePosMinus();
    }
    else
    {
        Encoder_1.pulsePosPlus();
    }
}

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

/*

Configuration des moteurs avec le rapport de réduction et le nombre d'impulsions par tour
*/
void setupMotors(){
    Encoder_1.setPulse(8);
    Encoder_1.setRatio(46);
    Encoder_2.setPulse(8);
    Encoder_2.setRatio(46);

    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    // Réglage PWM à 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
}

/*

Réglage de la tension des moteurs. Limité par MAX_VOLTAGE
*/
void setMotorsVoltage(float voltage1, float voltage2){
    Encoder_1.setMotorPwm(constrain(voltage1, -MAX_VOLTAGE, MAX_VOLTAGE) * VOLTS_2_PWM);
    Encoder_2.setMotorPwm(constrain(voltage2, -MAX_VOLTAGE, MAX_VOLTAGE) * VOLTS_2_PWM);
}

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(1);
    gyro.begin();
    Wire.setClock(100000);
    setupMotors();
    Timer5.initialize(Ts);
    Timer5.attachInterrupt(Timer5ISR);
}

void loop()
{
    noInterrupts(); // désactiver les interruptions
    float angleCopy = angle;
    float angle_pointCopy = angle_point;
    float speed1Copy = speed1;
    float speed2Copy = speed2;
    float position1Copy = position1;
    float position2Copy = position2;
    long compTimeCopy = compTime;
    interrupts(); // réactiver les interruptions

    // Affichage des données de diagnostic
    Serial.print(" compTime: ");
    Serial.print(compTimeCopy);

    Serial.print(" overrun: ");
    Serial.print(overrun);

    Serial.print(" speed1: ");
    Serial.print(speed1Copy, 2);

    Serial.print(" speed2: ");
    Serial.print(speed2Copy, 2);

    Serial.print(" position1: ");
    Serial.print(position1Copy, 2);

    Serial.print(" position2: ");
    Serial.print(position2Copy, 2);

    Serial.print(" angle: ");
    Serial.print(angleCopy, 2);

    Serial.print(" angle_point: ");
    Serial.print(angle_pointCopy, 2);

    Serial.print(" targetX: ");
    Serial.print(targetX, 2);

    Serial.print(" targetY: ");
    Serial.print(targetY, 2);

    Serial.println();

    delay(10); // délai pour éviter une surcharge du port série
}
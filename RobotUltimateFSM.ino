#include "MeMegaPi.h"
#include <Wire.h>
#include <TimerFive.h>

#define BELT_PITCH 2.032 // distance between teeth
#define NTEETH 90.0 // number of teeth

#define RPM_2_MMS BELT_PITCH*NTEETH/60.0 // conversion of the speed from rpm to mm/s

#define VOLTS_2_PWM 255.0/12.0 // conversion voltage to pwm [-255 255]
#define MAX_VOLTAGE 9 // max allowed voltage in V

#define Ts 5000 // Sampling rate in µs

#define Tau 0.02089f
#define k 54.44f
float a = -1/Tau;
float b = k/Tau;

#define l1 382.96f
#define l2 41257.0f
#define l3 1010.2f
#define kI -1.3190f
#define kx 0.0276f
#define Kb 52.0f
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
float Te = (Ts/1000)/1000.0f;

volatile float angle = 0; // actual heading in deg
volatile float angle_point = 0;
volatile float speed1 = 0; // actual speed in mm/s
volatile float speed2 = 0;
volatile float position1 = 0; // actual position in mm
volatile float position2 = 0; // actual position in mm
float u1 = 0; // control signals in V
float u2 = 0;
volatile long compTime = 0; // actual computation time of the critical loop
volatile short overrun = 0;
volatile float ref1 = 0;
volatile float ref2 = 0;
volatile float refAngle = 0;

// Cible à atteindre (en mm)
const float targetX = 2000; // distance sur l'axe X
const float targetY = 2000; // distance sur l'axe Y

// Vitesse de référence (mm/s)
float vxref = 0;
float vyref = 0;

// Position actuelle (en mm)
float x = 0;
float y = 0;

// Vitesse précédente
float vxref_pres = 0;
float vyref_pres = 0;

MeGyro gyro; // gyroscope object instanciation
MeEncoderOnBoard Encoder_1(SLOT1); // motor with encoder object instanciation
MeEncoderOnBoard Encoder_2(SLOT2);

/*

routine that is called every 5ms by the ISR routine
*/
void Update5ms()
{
    UpdateSensors();
    UpdateControl();
    UpdateActuators();
}

float calcU(float u) {
    if (u > 9.0f)
        return 9.0f;
    else if (u < -9.0f)
        return -9.0f;
    return u;
}

void UpdateSensors(){
    float angle_anc = angle;
    gyro.update(); // update the gyroscope state
    angle = gyro.getAngleZ(); // get the estimated heading in deg
    angle_point = (angle - angle_anc) / Te;
    Encoder_1.loop(); // update the encoders state
    Encoder_2.loop();

    speed1 = Encoder_1.getCurrentSpeed() * RPM_2_MMS; // compute the speed in mm/s
    speed2 = Encoder_2.getCurrentSpeed() * RPM_2_MMS;

    position1 = Encoder_1.getCurPos() * BELT_PITCH * NTEETH / 360.0f; // compute the position in mm
    position2 = Encoder_2.getCurPos() * BELT_PITCH * NTEETH / 360.0f;
}

void UpdateControl()
{
    // Mettre à jour les vitesses de référence pour atteindre la cible
    vxref_pres = vxref;
    vyref_pres = vyref;

    vxref = 0.0333 * (targetX - x);
    vyref = 0.0333 * (targetY - y);

    x += (vxref + vxref_pres)/2 * Te;
    y += (vyref + vyref_pres)/2 * Te;

    float l = 175 / 2 * 85 / 100;
    float L = 124.5f;
    float refSpeedChenille = cos(angle * PI / 180) * vxref + sin(angle * PI / 180) * vyref;
    refAngle = (-sin(angle * PI / 180) / L) * vxref + (cos(angle * PI / 180) / L) * vyref;

    // calcul diff chenille
    float refSpeedDifference = (refAngle) * L;

    // maj ref moteur
    ref1 = refSpeedChenille + refSpeedDifference;
    ref2 = refSpeedChenille - refSpeedDifference;

    // Update FSM for motor 1
    e1 = position1 - d01;
    float d01temp = d01 + Te * (v01 + e1 * l1);
    float v01temp = v01 + Te * (a * v01 + b * u1 + b * p01 + l2 * e1);
    float p01temp = p01 + Te * l3 * e1;

    d01 = d01temp;
    v01 = v01temp;
    p01 = p01temp;

    // Update FSM for motor 2
    e2 = position2 - d02;
    float d02temp = d02 + Te * (v02 + e2 * l1);
    float v02temp = v02 + Te * (a * v02 + b * u2 + b * p02 + l2 * e2);
    float p02temp = p02 + Te * l3 * e2;

    d02 = d02temp;
    v02 = v02temp;
    p02 = p02temp;

    // Calculate control signals
    float xi01temp = xi01 + Te * (-ref1 - v01 + Kb * (calcU(u1) - u1));
    float u10 = -kI * xi01 - kx * v01;

    float xi02temp = xi02 + Te * (-ref2 - v02 + Kb * (calcU(u2) - u2));
    float u20 = -kI * xi02 - kx * v02;

    xi01 = xi01temp;
    xi02 = xi02temp;
    u1 = u10;
    u2 = u20;
}

void UpdateActuators(){
    setMotorsVoltage(u1, u2); // set the voltages
}

/*

routine that is called every 5ms by the timer 5
*/
void Timer5ISR(){
    static char executing = 0; // set to 1 when the update function is running
    if(executing) {
        overrun = 1;
        return;
    }
    else executing = 1; // if already running => overrun

    interrupts(); // enable the interrupts during the execution of this loop
    long startTime = micros();

    Update5ms();

    compTime = micros() - startTime;
    executing = 0;
}

/*

interruption routine for encoder pulse counting (quadrature)
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

Setup the motors with the reduction ratio and number of pulses per turn
*/
void setupMotors(){
    Encoder_1.setPulse(8);
    Encoder_1.setRatio(46);
    Encoder_2.setPulse(8);
    Encoder_2.setRatio(46);

    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    // Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
}

/*

Sets the voltage to the motor. Limited by MAX_VOLTAGE
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
    noInterrupts();
    float angleCopy = angle;
    float angle_pointCopy = angle_point;
    float speed1Copy = speed1;
    float speed2Copy = speed2;
    float position1Copy = position1;
    float position2Copy = position2;
    long compTimeCopy = compTime;
    interrupts();

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

    delay(10);
}

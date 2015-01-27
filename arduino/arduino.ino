// (c) Author: Elijas (2015) // github.com/Elijas //
#include <Timer.h>
#include <NewPing.h>
#include "Motor.h"

#define PIN_LSONAR_TRG 22
#define PIN_LSONAR_ECH 23
#define PIN_CSONAR_TRG 26
#define PIN_CSONAR_ECH 27
#define PIN_RSONAR_TRG 30
#define PIN_RSONAR_ECH 31
#define SONAR_MAX_DISTANCE 200 //cm
#define PIN_LMOTOR_FWD 2
#define PIN_LMOTOR_BWD 3
#define PIN_RMOTOR_FWD 11
#define PIN_RMOTOR_BWD 12
#define MOTOR_LOWER_LIMIT 110 /MOTOR_STEP_SIZE
#define MOTOR_UPPER_LIMIT 200 /MOTOR_STEP_SIZE
#define MOTOR_STEP_SIZE 5
#define MOTOR_DELAY_STEP_UPDATE 30 //ms
#define INTERRUPTNO_LENCODER 4 //interrupt-pin(Mega2560): 0-2 1-3 2-21 3-20 4-19 5-18
#define INTERRUPTNO_RENCODER 5

Timer timer;

NewPing sonar[] = {
    NewPing(PIN_LSONAR_TRG, PIN_LSONAR_ECH, SONAR_MAX_DISTANCE),
    NewPing(PIN_CSONAR_TRG, PIN_CSONAR_ECH, SONAR_MAX_DISTANCE),
    NewPing(PIN_RSONAR_TRG, PIN_RSONAR_ECH, SONAR_MAX_DISTANCE)
};

Motor motor[] = {
    Motor(0, PIN_LMOTOR_FWD, PIN_LMOTOR_BWD, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT, MOTOR_STEP_SIZE, MOTOR_DELAY_STEP_UPDATE, &timer),
    Motor(1, PIN_RMOTOR_FWD, PIN_RMOTOR_BWD, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT, MOTOR_STEP_SIZE, MOTOR_DELAY_STEP_UPDATE, &timer)
};

volatile int encoder[2];
void ISR_encoder0() {encoder[0]++;}
void ISR_encoder1() {encoder[1]++;}

/*quickref
motor[0].set(0);
sonar[0].ping_cm()
sonar->convert_cm(sonar[0].ping_median(5))
sonar[0].check_timer(); //Check if ping has returned within the set distance limit.
encoder[0]
*/

void step1() {
    Serial.println(encoder[0]);
}
void step2() {
    
}



void setup() {
    //COMMS
    Serial.begin(9600);

    //LCD
    pinMode(10,OUTPUT); //backlight control
    
    //ENCODERS
    attachInterrupt(INTERRUPTNO_LENCODER, ISR_encoder0, RISING);
    attachInterrupt(INTERRUPTNO_RENCODER, ISR_encoder1, RISING);

    //TIMER
    timer.every(1000, step1);
    timer.after(2000, step2);
}
void loop() {timer.update();}


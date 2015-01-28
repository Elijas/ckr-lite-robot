// (c) Author: Elijas (2015) // github.com/Elijas //
#include <Timer.h>
#include <NewPing.h>
#include "Motor.h"

//==========================
//         MACROS
//==========================
//SONARS
#define PIN_LSONAR_TRG 22
#define PIN_LSONAR_ECH 23
#define PIN_CSONAR_TRG 26
#define PIN_CSONAR_ECH 27
#define PIN_RSONAR_TRG 30
#define PIN_RSONAR_ECH 31
#define SONAR_MAX_DISTANCE 200 //cm
//MOTORS
#define PIN_LMOTOR_FWD 2
#define PIN_LMOTOR_BWD 3
#define PIN_RMOTOR_FWD 11
#define PIN_RMOTOR_BWD 12
#define MOTOR_LOWER_LIMIT 110 /MOTOR_STEP_SIZE
#define MOTOR_UPPER_LIMIT 200 /MOTOR_STEP_SIZE
#define MOTOR_STEP_SIZE 10
#define MOTOR_DELAY_STEP_UPDATE 30 //ms
//ENCODERS
#define INTERRUPTNO_LENCODER 4 //quickref(Mega2560): interrupt-pin: 0-2 1-3 2-21 3-20 4-19 5-18
#define INTERRUPTNO_RENCODER 5
//COMMS
#define SerialBT Serial2 //quickref(Mega2560): SerialNo-RXpin-TXpin: 1-19-18 2-17-16 3 15-14

//==========================
//   OBJECTS AND FUNCTIONS
//==========================
//TIMER
Timer timer;

//SONARS
NewPing sonar[] = {
    NewPing(PIN_LSONAR_TRG, PIN_LSONAR_ECH, SONAR_MAX_DISTANCE),
    NewPing(PIN_CSONAR_TRG, PIN_CSONAR_ECH, SONAR_MAX_DISTANCE),
    NewPing(PIN_RSONAR_TRG, PIN_RSONAR_ECH, SONAR_MAX_DISTANCE)
};

//MOTORS
Motor motor[] = {
    Motor(0, PIN_LMOTOR_FWD, PIN_LMOTOR_BWD, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT, MOTOR_STEP_SIZE, MOTOR_DELAY_STEP_UPDATE, &timer),
    Motor(1, PIN_RMOTOR_FWD, PIN_RMOTOR_BWD, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT, MOTOR_STEP_SIZE, MOTOR_DELAY_STEP_UPDATE, &timer)
};

//ENCODERS
volatile int encoder[2];
void ISR_encoder0() {encoder[0]++;}
void ISR_encoder1() {encoder[1]++;}

//==========================
//          ACTIONS
//==========================
/*quickref
motor[0].set(0);
sonar[0].ping_cm()
sonar->convert_cm(sonar[0].ping_median(5))
sonar[0].check_timer(); //Check if ping has returned within the set distance limit.
encoder[0]
*/

int spd = 0;
int m = 0;

void pollSerialBT() {
    if (SerialBT.available() > 0) {
        char c = SerialBT.read();
        if (c == 'S') {
            motor[m].set(0);
        }
        else if (c == 'F') {
            motor[m].set(100 + spd*10);
        }
        else if (c == 'B') {
            
        }
        else if (c == 'L') {
            encoder[0] = 0;
            encoder[1] = 0;
        }
        else if (c == 'R') {
            m = !m;
        }
        else if (0 <= c-'0' &&  c-'0' <= 9) {
            spd=c-'0';
        }
        else if (c == 'q') {
            
        }
        //Serial.write(c);
    }
}

void action2() {
    Serial.println(encoder[m]);
}
void action3() {
    
}
void action4() {
    
}

void setup() {
    //COMMS
    Serial.begin(9600);
    Serial2.begin(9600);
    timer.every(80, pollSerialBT);

    //LCD
    pinMode(10,OUTPUT); //backlight control
    
    //ENCODERS
    attachInterrupt(INTERRUPTNO_LENCODER, ISR_encoder0, RISING);
    attachInterrupt(INTERRUPTNO_RENCODER, ISR_encoder1, RISING);

    //TIMER
    timer.every(1000, action2);
    //timer.after(2000, action2);
}
void loop() {timer.update();}


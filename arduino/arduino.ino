// (c) Author: Elijas (2015) // github.com/Elijas //
#include <Timer.h>
#include <NewPing.h>
#include <LiquidCrystal.h>
#include "Motor.h"

//==========================
//          MACROS
//==========================
//SONARS
#define PIN_LSONAR_TRG 22
#define PIN_LSONAR_ECH 23
#define PIN_CSONAR_TRG 26
#define PIN_CSONAR_ECH 27
#define PIN_RSONAR_TRG 30
#define PIN_RSONAR_ECH 31
#define SONAR_MAX_DISTANCE 20 //cm
//MOTORS
#define PIN_LMOTOR_FWD 2
#define PIN_LMOTOR_BWD 3
#define PIN_RMOTOR_FWD 11
#define PIN_RMOTOR_BWD 12
#define MOTOR_LOWER_LIMIT 130 /MOTOR_STEP_SIZE
#define MOTOR_UPPER_LIMIT 250 /MOTOR_STEP_SIZE
#define MOTOR_STEP_SIZE 10
#define MOTOR_DELAY_STEP_UPDATE 30 //ms
//ENCODERS
#define INTERRUPTNO_LENCODER 4 //quickref(Mega2560): interrupt-pin: 0-2 1-3 2-21 3-20 4-19 5-18
#define INTERRUPTNO_RENCODER 5
//MOVEMENT
#define MOVEMENT_MIN_POWER 130
#define MOVEMENT_EXTRA_POWER_MULTIPLIER 10
#define AUTONOMOUS_EXTRAMOVES_BACK 10
#define AUTONOMOUS_EXTRAMOVES_LEFT 10
#define AUTONOMOUS_EXTRAMOVES_RIGHT 10
//COMMS
#define SerialBT Serial2 //quickref(Mega2560): SerialNo-RXpin-TXpin: 1-19-18 2-17-16 3 15-14
//LCD
#define PINS_LCD 8, 9, 4, 5, 6, 7

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
volatile int encoder[] = {0,0};
void ISR_encoder0() {encoder[0]++;}
void ISR_encoder1() {encoder[1]++;}

//MOVEMENT
const int minPower = MOVEMENT_MIN_POWER;
int extraPower(0);
const int extraPowerMultiplier = MOVEMENT_EXTRA_POWER_MULTIPLIER;

void moveForward();
void moveBackward();
void spinLeftward();
void spinRightward();
void moveStop();

bool autonomousModeIsOn = false;
void moveAutonomously();
struct ExtraAvoidanceMovements { //After no more obstacles are detected, do these steps to move even further from the previously found obstacles
    bool active;
    int b; //steps to do: backward
    int l; //             leftward
    int r; //             rightward
    ExtraAvoidanceMovements() : active(false), b(0), l(0), r(0) {}
} extraMoves;
void doExtraAvoidanceMovements();

//COMMS
void readSerialBT();

//LCD
LiquidCrystal lcd(PINS_LCD);
void updateLCD();

//==========================
//      MAIN STRUCTURE
//==========================
void setup() {
    //ENCODERS
    attachInterrupt(INTERRUPTNO_LENCODER, ISR_encoder0, RISING);
    attachInterrupt(INTERRUPTNO_RENCODER, ISR_encoder1, RISING);

    //MOVEMENT
    timer.every(100, moveAutonomously);

    //COMMS
    Serial.begin(9600);
    Serial2.begin(9600);
    timer.every(80, readSerialBT);

    //LCD
    pinMode(10,INPUT); //backlight control
    lcd.begin(16, 2);
    timer.every(2000, updateLCD);
}
void loop() {timer.update();}

//==========================
//   FUNCTION DEFINITIONS
//==========================
//MOVEMENT
void moveForward() {
    motor[0].set(int(1*(minPower + extraPower*extraPowerMultiplier)));
    motor[1].set(int(1*(minPower + extraPower*extraPowerMultiplier)));
    lcd.setCursor(0, 1); lcd.print("Forward");
}
void moveBackward() {
    motor[0].set(int(-1*(minPower + extraPower*extraPowerMultiplier)));
    motor[1].set(int(-1*(minPower + extraPower*extraPowerMultiplier)));
    lcd.setCursor(0, 1); lcd.print("Backward");
}
void spinLeftward() {
    motor[0].set(int(-1*(minPower + extraPower*extraPowerMultiplier)));
    motor[1].set(int(1*(minPower + extraPower*extraPowerMultiplier)));
    lcd.setCursor(0, 1); lcd.print("Left    ");
}
void spinRightward() {
    motor[0].set(int(1*(minPower + extraPower*extraPowerMultiplier)));
    motor[1].set(int(-1*(minPower + extraPower*extraPowerMultiplier)));
    lcd.setCursor(0, 1); lcd.print("Right   ");
}
void moveStop() {
    motor[0].set(0);
    motor[1].set(0);
    lcd.setCursor(0, 1); lcd.print("        ");
}

void moveAutonomously() {
    if (!autonomousModeIsOn) {
        return;
    }
    
    if (sonar[1].ping()) { //method returns zero if no obstacles were detected
        moveBackward();
        extraMoves.active = true;
        extraMoves.b = AUTONOMOUS_EXTRAMOVES_BACK;
        extraMoves.r = AUTONOMOUS_EXTRAMOVES_RIGHT; //will move back and right even further after no more obstacles are detected
    }
    else if (sonar[0].ping()) {
        spinRightward();
        extraMoves.active = true;
        extraMoves.r = AUTONOMOUS_EXTRAMOVES_RIGHT;
    }
    else if (sonar[2].ping()) {
        spinLeftward();
        extraMoves.active = true;
        extraMoves.l = AUTONOMOUS_EXTRAMOVES_LEFT;
    }
    else {
        if (extraMoves.active) doExtraAvoidanceMovements();
        else moveForward();    
    }
}

void doExtraAvoidanceMovements() {
    Serial.println("X");

    if      (extraMoves.b) {
        extraMoves.b--;
        moveBackward();
    }
    else if (extraMoves.l) {
        extraMoves.l--;
        spinLeftward();
    }
    else if (extraMoves.r) {
        extraMoves.r--;
        spinRightward();
    }
    
    if (!extraMoves.l && !extraMoves.b && !extraMoves.r) 
        extraMoves.active = false;
}

//COMMS
void readSerialBT() {
    if (SerialBT.available() > 0) {
        char c = SerialBT.read();
        if (c == 'S') {
            moveStop();
        }
        else if (c == 'F') {
            if (!autonomousModeIsOn) moveForward();
        }
        else if (c == 'B') {
            if (!autonomousModeIsOn) moveBackward();
        }
        else if (c == 'L') {
            if (!autonomousModeIsOn) spinLeftward();
        }
        else if (c == 'R') {
            if (!autonomousModeIsOn) spinRightward();
        }
        else if ('0' <= c && c <= 9+'0') {
            extraPower = c-'0';
        }
        else if (c == 'q') {
            extraPower = 10;
        }
        else if (c == 'x') {
            autonomousModeIsOn = false;
            moveStop();
        }
        else if (c == 'X') {
            autonomousModeIsOn = true;
        }
        else if (c == 'v' || c == 'V') {
            encoder[0] = encoder[1] = 0;        
        }
    }
}

//LCD
void updateLCD() {
    lcd.print("       "); lcd.setCursor(0, 0); //clear screen
    lcd.print(encoder[0]);
    lcd.print(' ');
    lcd.print(encoder[1]);
}

/***Quick reference***
motor[0].set(0);
sonar[0].ping_cm()
sonar->convert_cm(sonar[0].ping_median(5))
sonar[0].check_timer() //Check if ping has returned within the set distance limit.
sonar[1].ping()
*/


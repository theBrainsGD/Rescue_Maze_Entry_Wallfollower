#include "SharpIR.h"
#include "LiquidCrystal_I2C.h"

#include "Engine.h"
#include "Util.h"
#include "Vars.h"



// Objektinitialisierung
LiquidCrystal_I2C lcd(LCDAddress, LCDCols, LCDRows);

SharpIR sharpLeftBack(IRLeftBack, 430);
SharpIR sharpFrontRight(IRFrontRight, 430);
SharpIR sharpFrontLeft(IRFrontLeft, 1080);
SharpIR sharpLeftFront(IRLeftFront, 430);


// Variablen

int valueLB = 0;
int valueFR = 0;
int valueFL = 0;
int valueLF = 0;

int graySclL = 0;
int graySclR = 0;
volatile int encoderLeft = 0;
volatile int encoderRight = 0;


void setup() {
Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LEDRed, OUTPUT);
  pinMode(LEDGreen, OUTPUT);
  pinMode(LightSensor, INPUT);
  pinMode(Piezo, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(18), countEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), countEncoderLeft, CHANGE);

  engineForward();
}

void loop() {
readSensors();


  //überprüfe immer zuerst, ob vorne ein Wand ist!
  if (valueFL < 9 && valueFR < 9) {
    Serial.print("FrontLeft: ");
    Serial.print(valueFL);
    Serial.print(" FrontRight: ");
    Serial.println(valueFR);
    lcd.clear();
    lcd.print("Wand");
    // wir müssen uns nach rechts drehen
    engineLeftForward();
    analogWrite(enb, 255);
    engineRightBackward();
    analogWrite(ena, 255);
    delay(1300);
    lcd.clear();
  }


  // Wenn die linken Sensoren kleiner als 9 sind müssen wir
  // von der linken Wand wegfahren
  // Wenn der linke Sensor kleiner ist als 13 und größer ist
  // als 10, dann soll wieder dichter zur Wand gefahren werden
  lcd.clear();
  if (valueLF < 9) {
    lcd.print("nach rechts");
    engineForward();
    analogWrite(ena, 140);
    analogWrite(enb, 255);
  } else if (valueLF < 13 && valueLF > 10) {
    lcd.print("weiter");
    engineForward();
    analogWrite(ena, 255);
    analogWrite(enb, 255);
  } else {
    lcd.print("links");
    engineForward();
    analogWrite(ena, 255);
    analogWrite(enb, 140);
  }

  // Wenn der linke vordere Sensor keine Wand mehr hat,
  // dann muss nach links abgebogen werden!
  if (valueLF > 17) {
    lcd.clear();
    lcd.print("leer");
    //    while (valueLB < 17) {
    //      readSensors();
    //      engineForward();
    //      analogWrite(ena, 255);
    //      analogWrite(enb, 255);
    //    }
    engineForward();
    analogWrite(ena, 255);
    analogWrite(enb, 255);

    delay(700);



    //wir sind in das Feld eingefahren
    // Drehen!
    lcd.clear();
    lcd.print("drehen");
    // wir müssen uns nach links drehen

    while (encoderLeft < 500 && encoderRight < 500) {

      if (encoderLeft < 500) {
        engineRightForward();
        analogWrite(ena, 255);
      } else engineRightStop();
      if (encoderLeft < 500) {
        engineLeftBackward();
        analogWrite(enb, 255);
      } else engineLeftStop();

    }
    lcd.clear();

    // solange der Hinter Sensor keine Wand sieht vorwärts
    while (valueLB > 17) {
      readSensors();
      if (valueFL < 7) {
        break;
      }
      lcd.clear();
      lcd.print("Ich suche");
      Serial.println(valueLB);
      readSensors();
      engineForward();
      analogWrite(ena, 255);
      analogWrite(enb, 255);
    }
  }
}



void readSensors() {
  valueLB = sharpLeftBack.distance();
  valueFR = sharpFrontRight.distance();
  valueFL = sharpFrontLeft.distance();
  valueLF = readLeftFront(sharpLeftFront);

  graySclL = analogRead(grSclL);
  graySclR = analogRead(grSclR);

  if (testForVictim()) {
    analogWrite(Piezo,  15);
  } else {
    analogWrite(Piezo,  0);
  }
}





void countEncoderLeft() {
  encoderLeft++;
}

void countEncoderRight() {
  encoderRight++;
}

void resetEncoder() {
  encoderLeft = 0;
  encoderRight = 0;
}

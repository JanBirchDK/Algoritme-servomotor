/*
 * Projekt: Overkørsel til modeljernbane
 * Produkt: Algoritme til vejbom
 * Version: 1.0
 * Type: Program
 * Programmeret af: Jan Birch
 * Opdateret: 20-11-2022
 * GNU General Public License version 3
 * This file is part of Algoritme til vejbom.
 * 
 * "Algoritme til vejbom" is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * "Algoritme til vejbom" is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with "Algoritme til servomotor".  If not, see <https://www.gnu.org/licenses/>.
 *  
 * Noter: 
 * Se koncept og specifikation for en detaljeret beskrivelse af programmet, formål og anvendelse.
 */

// Specifikationer for  servomotor 
struct t_MotorSpecs {
  int PulseWidthMin;  // Motorens mindste pulsbredde som giver mindste vinkel
  int PulseWidthMax;  // Motorens største pulsbredde som giver største vinkel
  int AngleMin;       // Motorens mindste vinkel
  int AngleMax;       // Motorens største vinkel
};

//---------- Opdateret klasse som senere skal indbygges i bibliotek

#include <Servo.h>
// Motor specifikationer
const struct t_MotorSpecs ESU51804Specs = {600, 2300, 0, 180};

// Beregning af værdi via opslag fra et interval over på et andet interval:
// Argumenter: x: opslagsværdi, in_min til in_max: opslagsinterval, out_min til out_max: beregningsinterval
// Returnerer: Beregnet værdi
float map_of_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Ansvar: Denne klasse varetager al funktion til styring af en servomotor.
// Udlæsning til pulsbreddemoduleret hardware port. Grænseflade til software.
// Seqs: Et bomdrev løber igennem 3 trin, når det går op eller ned
// seq: Bomdrevets trin
// servoPort: Portobjekt
// motorSpecs: Specifikationer for den motor der skal styres
// servoPeriod: Periodetid læst fra servo bibliotek
// upPW: Pulsbredde når bomdrev er i oppe
// downPW: Pulsbredde når bomdrev er i nede
// currentPW: Pulsbredde på et tidspunkt
// deltaPW: Ændring i pulsbredde per sampling
// value: Giver besked om fremtidig stilling af bom
// write(...): Indlæser besked om fremtidig stilling af bom
// startMotor(...): Sætter PWM variable indenfor grænser og kobler motor til port
// doPeriod(...): Gennemløb på tid
// checkConfig(...): Tjekker konfiguration
// sendOut(...): Sender ændring af pulsbredd til motor
class t_ServoMotor {
private:
  enum {STABLE, GOUP, GODOWN};
  byte seq;
  Servo servoPort;
  struct t_MotorSpecs motorSpecs;
  unsigned long servoPeriod;
  float upPW;
  float downPW;
  float currentPW;
  float deltaPW;
  bool value;
  void sendOut(void);
  bool checkConfig(int angleAdjust, int angleDiff, unsigned long barrierTime, int *upAngle, int *downAngle);
public:  
  t_ServoMotor(bool a_value=LOW): seq(STABLE) {}
  void write(bool a_value);
  void startMotor(byte pin, int angleAdjust, int angleDiff, unsigned long barrierTime, struct t_MotorSpecs a_motorSpecs);
  void doPeriod(void);
  bool readBarrierState(void);
};

void t_ServoMotor::sendOut(void) {
  if (servoPort.attached() == true) {
    servoPort.writeMicroseconds(round(currentPW));
  }
}

bool t_ServoMotor::checkConfig(int angleAdjust, int angleDiff, unsigned long barrierTime, int *upAngle, int *downAngle) {
// Tjek justering af vinkel
  bool isValid = false;
  *upAngle = constrain(angleAdjust, motorSpecs.AngleMin, motorSpecs.AngleMax);
  isValid = (*upAngle == angleAdjust);
  *downAngle = *upAngle+angleDiff;
  isValid = isValid && (*downAngle <= motorSpecs.AngleMax);
// Tjek tid for bom op eller ned
  isValid = isValid && (barrierTime > servoPeriod);
  return isValid;
}

void t_ServoMotor::write(bool a_value) {
  if (a_value != value) {
    value = a_value;
    sendOut();
  }
}

void t_ServoMotor::startMotor(byte pin, int angleAdjust, int angleDiff, unsigned long barrierTime, struct t_MotorSpecs a_motorSpecs) {
  int upAngle;     // Når bom er oppe
  int downAngle;   // Når bom er nede
  bool allowStart; // Viser om konfiguration må bruges
// Tjek konfiguration
  motorSpecs = a_motorSpecs;
  servoPeriod = REFRESH_INTERVAL/1000;  // Se kildekode for servo.h
  allowStart = checkConfig(angleAdjust, angleDiff, barrierTime, &upAngle, &downAngle);
// Definer pulsbredder
//  upPW = float(map(upAngle, motorSpecs.AngleMin, motorSpecs.AngleMax, motorSpecs.PulseWidthMin, motorSpecs.PulseWidthMax));
  upPW = map_of_float(float(upAngle), float(motorSpecs.AngleMin), float(motorSpecs.AngleMax), float(motorSpecs.PulseWidthMin), float(motorSpecs.PulseWidthMax)); 
//  downPW = float(map(downAngle, motorSpecs.AngleMin, motorSpecs.AngleMax, motorSpecs.PulseWidthMin, motorSpecs.PulseWidthMax));
  downPW = map_of_float(float(downAngle), float(motorSpecs.AngleMin), float(motorSpecs.AngleMax), float(motorSpecs.PulseWidthMin), float(motorSpecs.PulseWidthMax)); 
  deltaPW = float(downPW-upPW)/float(barrierTime/servoPeriod);
  currentPW = (value == HIGH)?upPW:downPW;
  Serial.print("Start motor? ");Serial.println(allowStart);
  Serial.print("upPW: ");Serial.println(upPW);
  Serial.print("downPW: ");Serial.println(downPW);
  Serial.print("deltaPW: ");Serial.println(deltaPW);
  Serial.print("currentPW: ");Serial.println(currentPW);
  if (allowStart == true) {
    servoPort.attach(pin);
    sendOut();  
  }
}

void t_ServoMotor::doPeriod(void) {
  switch (seq) {
    case STABLE:
      if ((value == HIGH) && (currentPW == downPW)) seq = GOUP;
      if ((value == LOW) && (currentPW == upPW)) seq = GODOWN;
    break;
    case GOUP:
      if (currentPW > upPW) {
        delay(servoPeriod);
        currentPW -= deltaPW;
        sendOut();
      }
      else seq = STABLE;      
    break;
    case GODOWN:
      if (currentPW < downPW) {
        delay(servoPeriod);
        currentPW += deltaPW;
        sendOut();
      }
      else seq = STABLE;
    break;
  }

}
//----------

// Specifikation af bom
const unsigned long BarrierTime = 16000; // Millisekunder
const unsigned long MaxBarrierTime = 20000; // Millisekunder
const int angleAdjust = 10;  // Bom justeret til lodret position
const int angleDiff = 90;    // Ændring af vinkel til bom nede
bool barrierState;           // Er bom oppe eller nede

// Opsætning af servomotor
const int servoPin = 9;         // Arduino pin til servomotor
t_ServoMotor ESU51804(LOW);      // Variabel til styring af servomotor

// Opsætning af betjening og lysmelding
const int ButtonPin = 2; // Port til knap
const int LEDPin = 7;    // Port til LED.
const unsigned long deBounceTime = 30; // Vent i msek og tjek igen om knap er trykket (forebygger kontaktprel)
unsigned long oneshotTime;  // Vent med næste omstilling af bom indtil bombevægelse er slut

// Opsætning af debug meddelelser
const int BaudRate = 9600;     // Seriel kommunikationshastighed

void setup() {
  // put your setup code here, to run once:
// Opsætning af betjening og meldinger
  pinMode(ButtonPin, INPUT_PULLUP); // Port bliver sat op til input konstant høj
  oneshotTime = millis();
  pinMode(LEDPin, OUTPUT);          // Port bliver sat op til output på Arduino
  digitalWrite(LEDPin, HIGH);       // LED viser aktiv Arduino og bom stabil
  Serial.begin(BaudRate);           // Aktiverer seriel kommunikation
// Start motor
  ESU51804.startMotor(servoPin, angleAdjust, angleDiff, BarrierTime, ESU51804Specs);
  barrierState = LOW;
}

void loop() {
  // put your main code here, to run repeatedly:

// Styring med knap og algoritme
  
  if (digitalRead(ButtonPin) == LOW) {
    delay(deBounceTime);
    if ((digitalRead(ButtonPin) == LOW) && (millis() > oneshotTime)) {
      digitalWrite(LEDPin, LOW);
      barrierState = (barrierState == HIGH)?LOW:HIGH;
      ESU51804.write(barrierState);  
      oneshotTime = millis()+MaxBarrierTime;
      Serial.print("Bom ");Serial.println(barrierState); 
    }  
  }
  if ((digitalRead(LEDPin) == LOW) && (millis() > oneshotTime)) digitalWrite(LEDPin, HIGH);
  ESU51804.doPeriod();
}

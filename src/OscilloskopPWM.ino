/*
 * Projekt: Overkørsel st. enkeltsporet strækning
 * Produkt: Tjek af nøjagtighed af pwm
 * Version: 1.0
 * Type: Program
 * Programmeret af: Jan Birch
 * Opdateret: 25-11-2022
 * GNU General Public License version 3
 * This file is part of PRODUKT.
 * 
 * "Tjek af nøjagtighed af pwm"  is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * "Tjek af nøjagtighed af pwm" is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with "Tjek af nøjagtighed af pwm".  If not, see <https://www.gnu.org/licenses/>.
 *  
 * Noter: 
 * I dette program bliver nøjagtighed af pwm undersøgt.
 * Det bliver målt på pwm udgang med oscilloskop.
 * Dette program bruger seriel monitor som input.
 */

#include <Servo.h>

// Opsætning af pwm
const int servoPin = 9;          // Arduino pin til pwm og servomotor
const int PulseWidthMin = 544;   // Motorens mindste pulsbredde som må blive målt
const int PulseWidthMax = 2400;  // Motorens største pulsbredde som må blive målt
const int PulseWidthStart = 1000;  // Motorens start pulsbredde
Servo servoPort;

// Opsætning af seriel kommunikation
const int BaudRate = 9600;     // Seriel kommunikationshastighed

// Opsætning af ventetid pr loop
const unsigned long loopTime = 3000;  // Vent med næste omstilling af bom indtil bombevægelse er slut

void setup() {
  // put your setup code here, to run once:
  servoPort.attach(servoPin);
  servoPort.writeMicroseconds(PulseWidthStart);
  Serial.begin(BaudRate);           // Aktiverer seriel kommunikation
  Serial.println("Pulsbredde i mikrosekunder? ");
}

void loop() {
  // put your main code here, to run repeatedly:
  int PWin;   // Modtager pulsbredde
  while (Serial.available() > 0)
  {
    PWin = Serial.parseInt();
// Tjek input
    if (PWin > 0) {
      if (PWin < PulseWidthMin) {
        Serial.print("Pulsbredde skal være > ");
        Serial.println(PulseWidthMin);
      }
      if (PWin > PulseWidthMax) {
        Serial.print("Pulsbredde skal være < ");
        Serial.println(PulseWidthMax);
      }
      if ((PWin >= PulseWidthMin) && (PWin <= PulseWidthMax)) {
        servoPort.writeMicroseconds(PWin);
        Serial.print(PWin);
        Serial.println(" mikrosekunder modtaget");
      }
      Serial.println("Pulsbredde i mikrosekunder? ");
    }
  }
  delay(loopTime);
}

// FRC Test Jig
// Integrates the DangerShield with an Arduino
//  
// You can also wire a slider potentiometer to A0, and digital switches to D10, D11, and D12.
// 
// Slider on A0 sets the speed of the jaguar (signal pin on pin 9) only when the button (D12) is
// pressed. Otherwise, the Jaguare will default to a neutral position.
//
// A spike relay is exercised using pins 
//
// Jaguar code is derived from BrainSTEM Education
// https://sites.google.com/site/0123icdesign/arduino_jaguar
//
// Written by: B. Huang, Feb. 2014

#include <avr/interrupt.h>
const int jagsigpin = 9; //pwm connection on pin 9
const int jagpwrpin = 8;//using pin 7 as the pwm power reference
const int jaggndpin = 7;//using pin 8 as the pwm ground reference
const int ledpin = 13;//using the standard LED as a status indicator
const int buffsize = 3;//determining the digits buffer size

/* Spike Relay works as follows:
// FWD when WHITE is HIGH, RED is LOW
// REV when WHITE is LOW, RED is HIGH
// OFF when WHITE is LOW, RED is LOW
// BRAKE/OFF when WHITE is HIGH, RED is HIGH
*/
const int spikesigpin = 5; // white wire -- "signal" wire (FWD) signal on Spike
const int spikepwrpin = 4; // red wire -- "power" wire (REV) signal on Spike
const int spikegndpin = 3; // using pin 3 as the pwm ground reference

int value = 47;// decimal 47 corresponds with the pwm midpoint
int minvalue = 16;// decimal 16 (empirically tested) corresponds with the pwm full reverse
int maxvalue = 78; //decimal 78 (empirically tested) corresponds with the pwm full forward

boolean ledstatus = LOW; //initialize the LED status

void setup()
{
  //set the PWM frequency to be 122Hz
  //  setPwmFrequency(9, 2); // sets PWM frequency divisor to 2
  TCCR1B = TCCR1B & 0b11111000 | 0x04;

  //setup the pins to be used with the PWM
  pinMode(jagsigpin, OUTPUT);
  pinMode(jagpwrpin, OUTPUT);
  pinMode(jaggndpin, OUTPUT);

  pinMode(spikesigpin, OUTPUT);
  pinMode(spikepwrpin, OUTPUT);
  pinMode(spikegndpin, OUTPUT);

  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  //setup the LED status pin
  pinMode(ledpin, OUTPUT);

  //define the pwm reference voltages
  digitalWrite(jagpwrpin,HIGH);
  digitalWrite(jaggndpin,LOW);
  digitalWrite(spikegndpin,LOW);
  //start the serial interface... 9600 is arbitrary
  Serial.begin(9600);

  //set the pwm to midscale so the jaguar starts in a friendly state
  analogWrite(jagsigpin,value);
}

void loop()
{
  int sliderVal = map(analogRead(A0), 0, 1023, minvalue, maxvalue);
  int buttonVal =digitalRead(12); 
  if(buttonVal == LOW)
  {
    analogWrite(jagsigpin,sliderVal);
    Serial.print("Setting Jaguar Speed: ");
    Serial.println(sliderVal);
  }
  else
    analogWrite(jagsigpin, value);  // value is the midpoint 
  int buttonVal1 = digitalRead(10);
  int buttonVal2 = digitalRead(11);
  digitalWrite(spikesigpin, buttonVal1);
  digitalWrite(spikepwrpin, buttonVal2);
}


/**
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 64: 
      mode = 0x03; 
      break;
    case 256: 
      mode = 0x04; 
      break;
    case 1024: 
      mode = 0x05; 
      break;
    default: 
      return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } 
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } 
  else if(pin == 3 || pin == 11) {
    switch(divisor) {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 32: 
      mode = 0x03; 
      break;
    case 64: 
      mode = 0x04; 
      break;
    case 128: 
      mode = 0x05; 
      break;
    case 256: 
      mode = 0x06; 
      break;
    case 1024: 
      mode = 0x7; 
      break;
    default: 
      return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


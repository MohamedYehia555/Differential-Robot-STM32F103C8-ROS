#include <Arduino.h>
#include "Cytron_setup.h"
// This shoud go in constructor but there is a silly limitation with
// Arduino and calling pinMode in a constuctor.  See the link below
// http://wiki.stm32duino.com/index.php?title=API#Important_information_about_global_constructor_methods
void MotoCytron::setup()
{
  // Initialize digital pins as outputs

    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM2,  OUTPUT);
  // Initialize braked and enabled
    digitalWrite(PWM1 ,LOW);
    digitalWrite(PWM2 ,LOW);
}
void MotoCytron::motorOff(byte motor)
{
  // Initialize braked
    digitalWrite(PWM1 ,LOW);
    digitalWrite(PWM2 ,LOW);
}
/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between 0 and 255, higher the number, the faster
 */
void MotoCytron::motorGo(byte motor, byte direct, int pwm)
{
{
  if (motor == 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
       if (direct <=1){
        digitalWrite(DIR1, LOW);
      }
      // Set inB[motor]
      else if (direct==2) {
        digitalWrite(DIR1, HIGH);
      }
      
      analogWrite(PWM1, pwm);
    }
  }
    else if (motor == 0){
        if (direct <=4)
    {
      // Set inA[motor]
       if (direct <=1){
        digitalWrite(DIR2, HIGH);
        }
      // Set inB[motor]
      else if (direct==2) {
        digitalWrite(DIR2, LOW);
      }
      analogWrite(PWM2, pwm);
    } 
 }  
}
}

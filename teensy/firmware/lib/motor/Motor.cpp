#include "Motor.h"

Controller::Controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB):
    motor_driver_(motor_driver),
    pwm_pin_(pwm_pin),
    motor_pinA_(motor_pinA),
    motor_pinB_(motor_pinB)
{
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
 
}

void Controller::spin(int lr, int pwm)
{
   
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, LOW);
                if (lr == 1){
                digitalWrite(motor_pinB_, HIGH);
		}
                else {digitalWrite(motor_pinB_, LOW);}
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
		if (lr == 1){
                digitalWrite(motor_pinB_, LOW);
		}
                else {digitalWrite(motor_pinB_, HIGH);}
            }
            analogWrite(pwm_pin_, abs(pwm));

    
}

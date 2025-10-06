/*
 * Control_Functions.h
 *
 *  Created on: Sep 25, 2025
 *      Author: Abdelaziz Hassan
 */

#ifndef INC_CONTROL_FUNCTIONS_H_
#define INC_CONTROL_FUNCTIONS_H_

#include "main.h"
#include "shared_variables.h"

//machine_init : get the system real dimention in CM to use in macine theta
void Machine_init(float d, float e, float f, float g);

/*Machine_theta : converts desired platform tilt (nx, ny)
 into the servo angle required for each leg to achieve that tilt.*/
float Machine_theta(int leg, float hz, float nx, float ny);

/* PID function :calculate the error from the actual ball postion from camera
from the set point "the position the ball should be in which is the center"*/
void PID_compute(int ballX, int ballY);

/* moveTo:compute servo angles based on error from PID function and make sure
 *  the output angle in the specified range (servoMin,servoMax) */
void moveTo(float hz, float nx, float ny,
            int *servoA_out, int *servoB_out, int *servoC_out);

//function to ensure to values in the specified range
float clamp(float val, float minVal, float maxVal) ;

//function to convert angle to pwm
void ServoControl(int angle1, int angle2, int angle3);

#endif /* INC_CONTROL_FUNCTIONS_H_ */

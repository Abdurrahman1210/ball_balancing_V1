/*
 * shared_variables.h
 *
 *  Created on: Sep 23, 2025
 *      Author: Abdelaziz Hassan
 */

#ifndef INC_SHARED_VARIABLES_H_
#define INC_SHARED_VARIABLES_H_

#include <stdint.h>
#include "main.h"

extern TIM_HandleTypeDef htim2;//so I can declare servo func in control file

//Constants for inverse kinematics
#define A 0
#define B 1
#define C 2

//constant for PID compute
#define Xval 0
#define Yval 1

//data receiving variables
#define buffer_size 16
extern volatile uint8_t buffer[buffer_size];
extern volatile uint8_t i;
extern volatile uint8_t datareceived;
extern int Xpos, Ypos;
extern const uint16_t Xsetpoint,Ysetpoint;
extern uint32_t timelapsed;

//servo variables
extern const uint8_t servoNeutralA , servoNeutralB , servoNeutralC ;
extern const uint8_t servoSignA , servoSignB , servoSignC;
extern const int servoMin , servoMax ;
extern int s1, s2, s3;

// machine dimension
extern float d, e, f, g;

//PID variables
extern const float kp ;
extern const float ki ;
extern const float kd ;

//array for x,y in pid compute func
extern float errorArr[2] ;
extern float errorPrev[2];
extern float integr[2]   ;
extern float deriv[2]    ;
extern float outVal[2]   ;

//loop period in sec
extern const float LOOP_PERIOD_S;

//baseline height
extern const float platformHz ;

//max slope
extern const float maxSlope ;

//value for initial theta func
extern const float initialval;

//calibration
extern float theta0A , theta0B , theta0C;


#endif /* INC_SHARED_VARIABLES_H_ */

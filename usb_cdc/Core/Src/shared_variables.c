/*
 * ------------------READ ME ---------------------------*
 * shared_variables.c
 * this file contain all variable declaration
 *
 *  Created on: Sep 23, 2025
 *      Author: Abdelaziz Hassan
 */
#include "shared_variables.h"

//data receiving variables
volatile uint8_t buffer[buffer_size];
volatile uint8_t i = 0;
volatile uint8_t datareceived = 0;
const uint16_t Xsetpoint=320, Ysetpoint=240;
uint32_t timelapsed = 0;
int Xpos, Ypos;

//servo variables
const uint8_t servoNeutralA = 74, servoNeutralB = 97, servoNeutralC = 83;
const uint8_t servoSignA = 1, servoSignB = 1, servoSignC = 1;
const int servoMin = 20, servoMax = 160;
int s1, s2, s3;

// system dimension
float d=5.70 ; //distance from the center of the base to any of its corners
float e=8.50 ; //distance from the center of the platform to any of its corners
float f=3.10 ; //length of link #1
float g=9.90; //length of link #2

//pid values
const float kp = 0.00040;
const float ki = 0.0;
const float kd = 0.00014;

//array for x,y in pid compute func
float errorArr[2] = {0,0};
float errorPrev[2]= {0,0};
float integr[2]   = {0,0};
float deriv[2]    = {0,0};
float outVal[2]   = {0,0};

//loop period "20ms"
const float LOOP_PERIOD_S = 0.02;

// baseline height
const float platformHz = 10.25;

// max slope
const float maxSlope = 0.6;

//value for initial theta func
const float initialval=0.0;

// calibration
float theta0A = 0.0, theta0B = 0.0, theta0C = 0.0;








/*
 *  * ------------------READ ME ---------------------------*
 * Control_Functions.c
 * this file contain all function implementation
 *
 *  Created on: Sep 25, 2025
 *      Author: Abdelaziz Hassan
 */
#include <math.h>
#include "Control_Functions.h"
#include "shared_variables.h"


//machine_init get the system real dimention in CM to use in macine theta
void Machine_init(float _d, float _e, float _f, float _g) {
    d = _d; e = _e; f = _f; g = _g;
}

/* Machine_theta : converts desired platform tilt (nx, ny)
 into the servo angle required for each leg to achieve that tilt.*/
float Machine_theta(int leg, float hz, float nx, float ny) {
	float nmag, nz;
	float x=0, y=0, z=0, mag, angle=0;

    // unit normal vector
    nmag = sqrt(nx*nx + ny*ny + 1.0);
    nx /= nmag;
    ny /= nmag;
    nz = 1.0 / nmag;

    switch (leg) {
        case A:  // Leg A
            y = d + (e/2.0) * (1.0 - ((nx*nx) + 3.0*nz*nz + 3.0*nz) /
                (nz + 1.0 - (nx*nx) + ((nx*nx*nx*nx) - 3.0*nx*nx*ny*ny) /
                ((nz+1.0) * (nz+1.0 - nx*nx))));
            z = hz + e*ny;
            mag = sqrt(y*y + z*z);
            angle = acos(y/mag) + acos((mag*mag + f*f - g*g)/(2.0*mag*f));
            break;

        case B:  // Leg B
            x = (sqrt(3.0)/2.0) * ( e*(1.0 - (nx*nx + sqrt(3.0)*nx*ny)/(nz+1.0)) - d );
            y = x / sqrt(3.0);
            z = hz - (e/2.0) * (sqrt(3.0)*nx + ny);
            mag = sqrt(x*x + y*y + z*z);
            angle = acos((sqrt(3.0)*x + y)/(-2.0*mag)) +
                    acos((mag*mag + f*f - g*g)/(2.0*mag*f));
            break;

        case C:  // Leg C
            x = (sqrt(3.0)/2.0) * ( d - e*(1.0 - (nx*nx - sqrt(3.0)*nx*ny)/(nz+1.0)) );
            y = -x / sqrt(3.0);
            z = hz + (e/2.0) * (sqrt(3.0)*nx - ny);
            mag = sqrt(x*x + y*y + z*z);
            angle = acos((sqrt(3.0)*x - y)/(2.0*mag)) +
                    acos((mag*mag + f*f - g*g)/(2.0*mag*f));
            break;
    }

    return angle * (180.0 / M_PI);
}

//function to ensure to values in the specified range
float clamp(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

//function to convert angle to pwm
void ServoControl(int angle1, int angle2, int angle3) {

    int servoAngle1 = 500 + (angle1 * 2000) / 180;
    int servoAngle2 = 500 + (angle2 * 2000) / 180;
    int servoAngle3 = 500 + (angle3 * 2000) / 180;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servoAngle1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servoAngle2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, servoAngle3);
}

/* PID function :calculate the error from the actual ball postion from camera
from the set point "the position the ball should be in which is the center"*/
void PID_compute(int ballX, int ballY) {
    errorPrev[Xval] = errorArr[Xval];
    errorPrev[Yval] = errorArr[Yval];

    // error = setpoint - position
    errorArr[Xval] = Xsetpoint - ballX;
    errorArr[Yval] = Ysetpoint - ballY;

    // integrator (trapezoidal method)
    integr[Xval] += (errorArr[Xval] + errorPrev[Xval]) * LOOP_PERIOD_S;
    integr[Yval] += (errorArr[Yval] + errorPrev[Yval]) * LOOP_PERIOD_S;

    // derivative
    deriv[Xval] = (errorArr[Xval] - errorPrev[Xval]) / LOOP_PERIOD_S ;
    deriv[Yval] = (errorArr[Yval] - errorPrev[Yval]) / LOOP_PERIOD_S ;

    // PID output
    outVal[Xval] = kp*errorArr[Xval] + ki*integr[Xval] + kd*deriv[Xval];
    outVal[Yval] = kp*errorArr[Yval] + ki*integr[Yval] + kd*deriv[Yval];

    // saturate outputs to safe slope
    outVal[Xval] = clamp(outVal[Xval], -maxSlope, maxSlope);
    outVal[Yval] = clamp(outVal[Yval], -maxSlope, maxSlope);

}

/* moveTo:compute servo angles based on error from PID function and make sure
 *  the output angle in the specified range (servoMin,servoMax) */
void moveTo(float hz, float nx, float ny,
             int *servoA_out, int *servoB_out, int *servoC_out) {

    if (nx >  maxSlope) nx =  maxSlope;
    if (nx < -maxSlope) nx = -maxSlope;
    if (ny >  maxSlope) ny =  maxSlope;
    if (ny < -maxSlope) ny = -maxSlope;

    float thetaA = Machine_theta(A, hz, nx, ny);
    float thetaB = Machine_theta(B, hz, nx, ny);
    float thetaC = Machine_theta(C, hz, nx, ny);

    int servoAngleA = servoNeutralA + servoSignA * (int)round(thetaA - theta0A);
    int servoAngleB = servoNeutralB + servoSignB * (int)round(thetaB - theta0B);
    int servoAngleC = servoNeutralC + servoSignC * (int)round(thetaC - theta0C);

    servoAngleA = clamp(servoAngleA, servoMin, servoMax);
    servoAngleB = clamp(servoAngleB, servoMin, servoMax);
    servoAngleC = clamp(servoAngleC, servoMin, servoMax);

    if (servoA_out) *servoA_out = servoAngleA;
    if (servoB_out) *servoB_out = servoAngleB;
    if (servoC_out) *servoC_out = servoAngleC;
}



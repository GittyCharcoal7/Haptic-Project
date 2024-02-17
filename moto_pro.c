/*
 * moto_pro.c
 *
 *  Created on: Apr 13, 2023
 *      Author: John
 */

// RGB LED Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD Interface
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   M1PWM5 (PF1) drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   M1PWM7 (PF3) drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   M1PWM6 (PF2) drives an NPN transistor that powers the blue LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <moto_pro.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"

// PortF masks
// #define RED_LED_MASK 2
// #define BLUE_LED_MASK 4
// #define GREEN_LED_MASK 8

// PortE masks
#define MOTOR_MASK 32

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize RGB
void initMoto()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure three LEDs
    GPIO_PORTE_DEN_R |= MOTOR_MASK; //RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTE_AFSEL_R |= MOTOR_MASK; //RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTE_PCTL_R &= ~GPIO_PCTL_PE5_M; //~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE5_M1PWM3; // | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;

    // Configure PWM module 1 to drive RGB LED
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_1_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 5)
    // PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM1_1_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 3 on PWM1, gen 1b, cmpb
    // PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 6 on PWM1, gen 3a, cmpa
    // PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 7 on PWM1, gen 3b, cmpb

    PWM1_1_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    // PWM1_3_LOAD_R = 1024;                            // (internal counter counts down from load value to zero)

    PWM1_1_CMPB_R = 0;                               // motor off (0=always low, 1023=always high)
    // PWM1_3_CMPB_R = 0;                               // green off
    // PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 1
    // PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
     PWM1_ENABLE_R = PWM_ENABLE_PWM3EN; // PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs
}

// void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
// {
//     PWM1_2_CMPB_R = red;
//     PWM1_3_CMPA_R = blue;
//     PWM1_3_CMPB_R = green;
// }

void setMoto(uint16_t motor)
{
    PWM1_1_CMPB_R = motor;
}






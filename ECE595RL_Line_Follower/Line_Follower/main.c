/**
 * @file main.c
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The main controller demonstrates a Line Follower robot without using a specific algorithm.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author Srushti Wadekar, Arshia P, Aaron Nanas
 *
 */

#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"

// Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL         2500
#define PWM_SWING           1000
#define PWM_MIN             (PWM_NOMINAL - PWM_SWING)
#define PWM_MAX             (PWM_NOMINAL + PWM_SWING)

// Initialize a global variable for Timer A1 to keep track of elapsed time in milliseconds
static uint32_t Timer_A1_ms_elapsed = 0;

// Declare global variables used to store line sensor position
static int32_t Line_Sensor_Position;

// Define the states for the Line Follower FSM
typedef enum
{
    MOVE_FORWARD = 0,
    MOVE_TO_LEFT = 1,
    MOVE_TO_RIGHT = 2
} Line_Follower_State;

Line_Follower_State current_state = MOVE_FORWARD;

/**
 * @brief Implements the FSM logic for the Line Follower robot based on sensor readings.
 *
 * @param None
 *
 * @return None
 */
void Line_Follower_FSM()
{
    // Check if the robot is at the center of the line (or very close to it)
    // Assign current_state to MOVE_TO_CENTER so that the robot will keep moving forward at the center
    if (Line_Sensor_Position > -47 && Line_Sensor_Position < 47)
    {
        current_state = MOVE_FORWARD;
    }

    // Check if the robot is on the left side of the line
    // Assign current_state to MOVE_TO_RIGHT to steer the robot to the right in order to move back to the center
    else if (Line_Sensor_Position >= 47 && Line_Sensor_Position < 332)
    {
        current_state = MOVE_TO_RIGHT;

        if (Line_Sensor_Position > -47 && Line_Sensor_Position < 47)
        {
            current_state = MOVE_FORWARD;
        }
    }

    // Check if the robot is on the right side of the line
    // Assign current_state to MOVE_TO_LEFT to steer the robot to the left in order to move back to the center
    else if (Line_Sensor_Position <= -47 && Line_Sensor_Position > -332)
    {
        current_state = MOVE_TO_LEFT;

        if (Line_Sensor_Position > -47 && Line_Sensor_Position < 47)
        {
            current_state = MOVE_FORWARD;
        }
    }

    // Otherwise, the robot will keep turning right at other positions (e.g. dead end)
    else
    {
        current_state = MOVE_TO_RIGHT;
    }
}

/**
 * @brief Implements the control logic for a Line Follower robot.
 *
 * This function controls the movement of the Line Follower robot based on
 * the current state determined by the FSM in the Line_Follower_FSM function.
 *
 * It performs the following actions based on the state:
 * - MOVE_FORWARD: Moves forward and changes the RGB LED's color to green
 * - MOVE_TO_LEFT: Turns left and changes the RGB LED's color to blue
 * - MOVE_TO_RIGHT: Turns right and changes the RGB LED's color to yellow
 *
 * @param None
 *
 * @return None
 */
void Line_Follower_Controller()
{
    switch(current_state)
    {
        case MOVE_FORWARD:
        {
            LED2_Output(RGB_LED_GREEN);
            Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }

        case MOVE_TO_LEFT:
        {
            LED2_Output(RGB_LED_BLUE);
            Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }

        case MOVE_TO_RIGHT:
        {
            LED2_Output(RGB_LED_YELLOW);
            Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }
    }
}

/**
 * @brief This function is the handler for the SysTick periodic interrupt with a rate of 1 kHz.
 *
 * The SysTick_Handler generates a periodic interrupt that calls the Line_Follower_Controller function.
 *
 * @param None
 *
 * @return None
 */
void SysTick_Handler(void)
{
    Line_Follower_Controller();
}

/**
 * @brief
 *
 * @param None
 *
 * @return None
 */
void Timer_A1_Periodic_Task(void)
{
    // Increment Timer_A1_ms_elapsed by 1 every time the Timer A1 periodic interrupt occurs
    Timer_A1_ms_elapsed++;

    // Start the process of reading the reflectance sensor array every 10 ms (i.e. 11, 21, 31, ...)
    if ((Timer_A1_ms_elapsed % 10) == 0)
    {
        Reflectance_Sensor_Start();
    }

    // Finish reading the reflectance sensor sensor array after 1 ms (i.e. 12, 22, 32, ...)
    if ((Timer_A1_ms_elapsed % 10) == 1)
    {
        uint8_t Line_Sensor_Binary_Value = Reflectance_Sensor_End();
        Line_Sensor_Position = Reflectance_Sensor_Position(Line_Sensor_Binary_Value);
        Line_Follower_FSM();
    }
}

int main(void)
{
    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED and the RGB LED on the MSP432 microcontroller
    LED1_Init();
    LED2_Init();

    // Initialize the buttons on the MSP432 microcontroller
    Buttons_Init();

    // Initialize the EUSCI_A0_UART module
    EUSCI_A0_UART_Init_Printf();

    // Initialize the motors
    Motor_Init();

    // Initialize the 8-Channel QTRX Reflectance Sensor Array module
    Reflectance_Sensor_Init();

    // Initialize the SysTick timer to generate periodic interrupts every 1 ms
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize Timer A1 periodic interrupts every 1 ms
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Enable the interrupts used by the modules
    EnableInterrupts();

    while(1)
    {

    }
}

//
// Created by adam slaymark on 05/05/2023.
//

#include "main.h"
#include "stdio.h"

// 2 second time for LED to be running
#define TIME_LIMIT 2000//ms

static void ToggleLED(uint8_t);

enum LED{
    GREEN,
    RED,
    OFF
};

static uint8_t Current_color;
static uint32_t start = 0;
static uint8_t start_flag = 0;


/**
  * @brief  Runs LED process.
  * @param  *P: Process structure
  */
// This could be handled by interrupts, however, as the CPU clock speed is high and toggling LED takes no time, this can be in the main thread.
void LED_Process(Process *P){
    // Check for change
    if (P->change_flag == 1){
        switch (P->YesNo) {
            case YES:
                puts("Yes Detected");
                if (start_flag == 0) {
                    ToggleLED(GREEN);
                    start = HAL_GetTick();
                    start_flag = 1;
                }
            case NO:
                puts("No Detected");
                if (start_flag == 0) {
                    ToggleLED(RED);
                    start = HAL_GetTick();
                    start_flag = 1;
                }
            case ERR:
                puts("Error Detected");
        }
        P->change_flag = 0;
    }
    if (start_flag == 1){
        if (HAL_GetTick() >= (start+TIME_LIMIT)){
            ToggleLED(Current_color);
            Current_color = OFF;
            start_flag = 0;
        }
    }

}
void ToggleLED(uint8_t C){

    switch (C) {
        case GREEN:
            HAL_GPIO_TogglePin(LD_USER1_GPIO_Port, LD_USER1_Pin);
            Current_color = GREEN;
            break;
        case RED:
            HAL_GPIO_TogglePin(LD_USER2_GPIO_Port, LD_USER2_Pin);
            Current_color = RED;
            break;
        default:
            break;
    }
}


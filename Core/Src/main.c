/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "audio_init.h"
#include <stdlib.h>
#include "retarget.h"
#include "wm8994.h"
#include "envelope.h"
#include "arm_math.h"

// import filters
#include "highPass_coef_6000Hz.h"
#include "lowPass_coef_2000Hz.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t    buff_pos;
    uint8_t     start_flag;
    uint8_t     init;
    uint8_t     YesNo;
}Process;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Envelope settings
#define THRESHOLD 0.007
#define MAX_AUDIO_SAMPLE_DUR 5 // in seconds
#define FRAME_LENGTH  (SAMPLE_RATE*MAX_AUDIO_SAMPLE_DUR/(BUFFER_SIZE/2))
#define PROCESS_BUFFER_SIZE ROUND_UP((FRAME_LENGTH*(BUFFER_SIZE/2)), 2)

//Yes No settings
#define CUTOFF 0.0032
#define YESMAX 1.368
#define NOMIN 0.000184

//#define XSTR(x) STR(x)
//#define STR(x) #x

//#pragma message "Value of process buffer: " XSTR(PROCESS_BUFFER_SIZE)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ProcessBuffer(int16_t*, Process*);
uint8_t YesNo(int16_t* );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Define buffer for right and left audio channels
static int16_t recordBuffer_R[BUFFER_SIZE / 2];
static int16_t recordBuffer_L[BUFFER_SIZE / 2];

__IO BufferStatusMessage_t bufferStatus;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */
    SCB_InvalidateICache();

    SCB->CCR |= (1 << 18); /* Enable branch prediction */
    __DSB();

    SCB_InvalidateICache();

    /* USER CODE END 1 */

    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();

    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    RetargetInit(&huart1);
    printf("Program Started!--------------------\n");

    //Initialise Audio -  Frequency selection must be defined by the wm9884 driver
    Audio_Init(SAMPLE_RATE, 85);

    Process process = {
            .init = 0,
            .start_flag = 0,
            .buff_pos = 0,
            .YesNo = 2,
    };

    envelope_alloc();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        // Update status message
//        if (process.init == 1 && process.start_flag == 0){
//            puts("Signal Received");
//            process.start_flag = 1;
//        } else if (process.init == 0 && process.start_flag == 1){
//            puts("signal sampling finished");
//            printf("command received: %d",process.YesNo);
//            process.start_flag = 0;
//        }

        switch (bufferStatus) {
            case BUFFER_STATUS_LOWER_HALF_FULL: {
                RX_LowerHalf(&recordBuffer_L[0],&recordBuffer_R[0], BUFFER_SIZE / 2);

                //do processing here
                printf("%d",recordBuffer_L[1]);
                puts("first Half");
                ProcessBuffer(&recordBuffer_L[0], &process);

                //TX_LowerHalf(&recordBuffer_L[0],&recordBuffer_R[0], BUFFER_SIZE / 2);
                break;
            }
            case BUFFER_STATUS_UPPER_HALF_FULL: {
                RX_UpperHalf(&recordBuffer_L[0],&recordBuffer_R[0], BUFFER_SIZE / 2);

                //do processing here
                puts("second Half");
                ProcessBuffer(&recordBuffer_L[0], &process);

                //TX_UpperHalf(&recordBuffer_L[0],&recordBuffer_R[0], BUFFER_SIZE / 2);
                break;
            }
        }
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 216;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
    */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

static int16_t audioProcessBuffer[PROCESS_BUFFER_SIZE];

// Main function for processing the buffer
void ProcessBuffer(int16_t* S, Process *P){
    float32_t tmp_m[BUFFER_SIZE/2] = {0};
    envelope(S,tmp_m);
    if (P->buff_pos > (PROCESS_BUFFER_SIZE)) {
        puts("Buffer overrun");
        return;
    }
    if (P->init == 0) {
        for (int i = 0; i < (BUFFER_SIZE / 2); i++) {
            printf("%f\n",tmp_m[i]);
            if (tmp_m[i] > THRESHOLD) {
                P->init = 1;
                puts("start");
                Error_Handler();
                break;
            }
        }
        for (int i=0; i < (BUFFER_SIZE/2);i++){
            audioProcessBuffer[P->buff_pos] = S[i];
            P->buff_pos++;
        }
    } else if (P->init == 1){
        puts("end");
        for (int i = 0; i < (BUFFER_SIZE / 2); i++) {
            if (tmp_m[i] < THRESHOLD) {
                //copy final part of buffer
                for (int d=0; d < (BUFFER_SIZE/2);d++){
                    audioProcessBuffer[P->buff_pos] = S[d];
                    P->buff_pos++;
                }
                P->buff_pos = 0;
                break;
            }
        }
        // Check if yes or no, write to struct
        P->YesNo = YesNo(audioProcessBuffer);
        printf("signal status: %d\n", P->YesNo);
        return;

    } else if(P->init == 1){ // Needs changing, already checking for init == 1 above. try and incorporate in same condition.
        puts("middle");
        for (int i=0; i < (BUFFER_SIZE/2);i++){
            audioProcessBuffer[P->buff_pos] = S[i];
            P->buff_pos++;
        }
    }
}

uint8_t YesNo(int16_t* buffer){
    //Create buffer for temporary float
    float32_t tmpBuffer[BUFFER_SIZE/2];
    // copy data to float buffer
    for (int i =0;i<(BUFFER_SIZE/2);i++){
        tmpBuffer[i] = (float32_t)buffer[i];
    }
    //Create filter buffer
    float32_t filter_buffer[BUFFER_SIZE/2];


    //HP filter the signal
    arm_biquad_cascade_df2T_f32(&highPass,tmpBuffer,filter_buffer,(BUFFER_SIZE/2));
    float32_t HP = 0;
    for (int i=0;i<(BUFFER_SIZE/2);i++){
        HP = (filter_buffer[i] * filter_buffer[i]) + HP;
    }
    memset(filter_buffer, 0, sizeof(float32_t) * (BUFFER_SIZE / 2));

    // LP filter the signal
    arm_biquad_cascade_df2T_f32(&lowPass,tmpBuffer,filter_buffer,(BUFFER_SIZE/2));
    float32_t LP = 0;
    for (int i=0;i<(BUFFER_SIZE/2);i++){
        LP = (filter_buffer[i] * filter_buffer[i]) + LP;
    }

    //calculate ratio
    float32_t ratio = HP/LP;

    if (ratio >= CUTOFF){
        if (ratio <= YESMAX) return 1; // return yes
        else return 2; // return error
    } else {
        if (ratio >= NOMIN) return 0; //return no
        else return 2; //return error
    }
}


//These functions must be included as DMA callback
void AUDIO_IN_HalfTransfer_CallBack(void) {
    //printf("AUDIO IN HALF");
    bufferStatus = BUFFER_STATUS_LOWER_HALF_FULL;
}

void AUDIO_IN_TransferComplete_CallBack(void) {
    //printf("AUDIO IN COMPLETE");
    bufferStatus = BUFFER_STATUS_UPPER_HALF_FULL;
}

void AUDIO_OUT_Error_CallBack(void) {
    puts("DMA Error");
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    puts("entered error state");
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

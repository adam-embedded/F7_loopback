/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Adam Slaymark.
  * All rights reserved.
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
#include "stdbool.h"
#include "audio_init.h"
#include <stdlib.h>
#include "retarget.h"
#include "wm8994.h"
#include "envelope.h"
#include "arm_math.h"
//#include "led_proc.h"

// import filters
#include "highPass_coef_6000Hz.h"
#include "lowPass_coef_2000Hz.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Envelope settings
#define THRESHOLD 2350

//Yes No settings
#define CUTOFF 0.0032
#define YESMAX 1.368
#define NOMIN 0.000184

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
void ProcessBuffer(int16_t *, Process *);
void YesNo(int16_t *, float32_t *, float32_t *, Process *);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Define buffer for right and left audio channels
static int16_t recordBuffer_R[BUFFER_SIZE / 2];
static int16_t recordBuffer_L[BUFFER_SIZE / 2];

__IO BufferStatusMessage_t bufferStatus;

//static int16_t audioProcessBuffer[PROCESS_BUFFER_SIZE];


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

    // Could be a global variable, however it is good to practice memory safety
    Process process = {
            .init = 0,
            .change_flag = 0,
            .buff_pos = 0,
            .YesNo = 2,
            .work = 0,
    };

    envelope_alloc();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        // Run LED function
        LED_Process(&process);


        switch (bufferStatus) {
            case BUFFER_STATUS_LOWER_HALF_FULL: {
                RX_LowerHalf(&recordBuffer_L[0], &recordBuffer_R[0], BUFFER_SIZE / 2);
                ProcessBuffer(&recordBuffer_L[0], &process);

#ifdef DEBUG
                TX_LowerHalf(&recordBuffer_L[0], &recordBuffer_R[0], BUFFER_SIZE / 2);
#endif
                bufferStatus = BUFFER_STATUS_IDLE;
                break;
            }
            case BUFFER_STATUS_UPPER_HALF_FULL: {
                RX_UpperHalf(&recordBuffer_L[0], &recordBuffer_R[0], BUFFER_SIZE / 2);
                ProcessBuffer(&recordBuffer_L[0], &process);

#ifdef DEBUG
                TX_UpperHalf(&recordBuffer_L[0], &recordBuffer_R[0], BUFFER_SIZE / 2);
#endif
                bufferStatus = BUFFER_STATUS_IDLE;
                break;
            }
            default:
                break;
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

static float32_t tmp_m[BUFFER_SIZE / 2] = {0};
static float32_t HP = 0.0f;
static float32_t LP = 0.0f;

// Main function for processing the buffer
void ProcessBuffer(int16_t *S, Process *P) {
    //uint32_t start = HAL_GetTick();
    // allocate temporary results buffer, make sure to fill buffer with zeros.
    envelope(S, tmp_m);

    //Check if process has init
    if (!P->init) {
        //puts("in zero");

        // Check if the threshold is greater than zero
        if (tmp_m[0] != 0) {
            // iterate through the buffer to check whether any value exceeds threshold
            for (int i = 0; i < (BUFFER_SIZE / 2); i++) {
                //printf("%f\n",tmp_m[i]);
                //Check threshold
                if (tmp_m[i] > THRESHOLD) {
                    //if greater, set init to true
                    P->init = true;
                    puts("start");
                    YesNo(&S[0], &HP, &LP, P);
                    break;
                }
            }
        }
    } else {
        puts("end");
        for (uint32_t i = 0; i < (BUFFER_SIZE / 2); i++) {
            if (tmp_m[i] < THRESHOLD) {
                puts("reached threshold for end");

                P->buff_pos = 0;
                P->work = 1;
                P->init = false;
                break;
            }
        }
        // Check if yes or no, write to struct
        YesNo(&S[0], &HP, &LP, P);
        //printf("signal status: %d\n", P->YesNo);
    }
    uint32_t end = HAL_GetTick();

    //printf("Elapsed Time for process: %lu", (end - start));
}

//Create buffer for temporary float
float tmpBuffer[BUFFER_SIZE / 2] = {0};

/**
  * @brief  Detects yes or no from signal.
  * @param  *buffer: input samples address
  * @param  *HP: High Pass value address
  * @param  *LP: Low Pass value address
  * @param  *P: Process structure address
  */
void YesNo(int16_t *buffer, float32_t *HP, float32_t *LP, Process *P) {
//Create filter buffer
    float32_t filter_buffer[BUFFER_SIZE / 2];

    // copy data to float buffer
    for (uint32_t i = 0; i < (BUFFER_SIZE / 2); i++) {
        tmpBuffer[i] = (float) buffer[i];
    }

//    //HP filter the signal
    arm_biquad_cascade_df2T_f32(
            &highPass,
            tmpBuffer,//(float*)buffer,
            filter_buffer,
            BUFFER_SIZE / 2);

    for (uint32_t i = 0; i < (BUFFER_SIZE / 2); i++) {
        // Sum and square sample values
        *HP = (filter_buffer[i] * filter_buffer[i]) + *HP;
    }
    memset(filter_buffer, 0, (sizeof(float32_t) * (BUFFER_SIZE / 2)));

    // LP filter the signal
    arm_biquad_cascade_df2T_f32(
            &lowPass,
            tmpBuffer,//(float*)buffer,
            filter_buffer,
            (BUFFER_SIZE / 2));

    for (uint32_t i = 0; i < (BUFFER_SIZE / 2); i++) {
        *LP = (filter_buffer[i] * filter_buffer[i]) + *LP;
    }

    // Do this once the recording has ended which is when envelope declares end of sample
    if (P->work == 1) {
        //calculate ratio
        float32_t ratio = *HP / *LP;
        P->work = 0;

        //Reset HP and LP to zero
        *HP = 0.0f;
        *LP = 0.0f;

        if (ratio >= CUTOFF) {
            if (ratio <= YESMAX) P->YesNo = YES; // return yes
            else P->YesNo = ERR; // return error
        } else {
            if (ratio >= NOMIN) P->YesNo = NO; //return no
            else P->YesNo = ERR; //return error
        }
        P->change_flag = 1;
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

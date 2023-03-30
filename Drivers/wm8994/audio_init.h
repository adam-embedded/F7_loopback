//
// Created by adam slaymark on 13/02/2023.
//

#ifndef F7_LOOPBACK_AUDIO_INIT_H
#define F7_LOOPBACK_AUDIO_INIT_H

#include "stdint.h"
#include "wm8994.h"

#define BUFFER_SIZE 1024//4096//2048//1024

#define SAMPLE_RATE AUDIO_FREQUENCY_32K


enum {
    BUFFER_STATUS_LOWER_HALF_FULL,
    BUFFER_STATUS_UPPER_HALF_FULL,
    BUFFER_STATUS_IDLE
};
typedef uint32_t BufferStatusMessage_t ;

void Audio_Init(uint32_t frequency, uint8_t volume);


void RX_LowerHalf(int16_t * sampleBuffer_L, int16_t * sampleBuffer_R, uint32_t num_samples);
void RX_UpperHalf(int16_t * sampleBuffer_L, int16_t * sampleBuffer_R, uint32_t num_samples);
void TX_LowerHalf(int16_t * sampleBuffer_L, int16_t * sampleBuffer_R, uint32_t num_samples);
void TX_UpperHalf(int16_t * sampleBuffer_L, int16_t * sampleBuffer_R, uint32_t num_samples);

void AUDIO_IN_TransferComplete_CallBack(void);
void AUDIO_IN_HalfTransfer_CallBack(void);
void AUDIO_OUT_Error_CallBack(void);

#endif //F7_LOOPBACK_AUDIO_INIT_H

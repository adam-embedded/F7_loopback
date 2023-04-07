//
// Created by adam slaymark on 13/02/2023.
//

#ifndef F7_LOOPBACK_AUDIO_INIT_H
#define F7_LOOPBACK_AUDIO_INIT_H

#include "stdint.h"
#include "wm8994.h"

#define BUFFER_SIZE 512//4096//2048//1024

//frame length 64 bytes, ie. samples are spaced 8 bytes apart
//sample size 2 bytes
//frame is every 8 bytes, data appears at the 1st and 4th, left and right channel

#define MY_DMA_BYTES_PER_FRAME 8
#define MY_DMA_BYTES_PER_MSIZE 2
#define MY_DMA_BUFFER_SIZE_BYTES BUFFER_SIZE * MY_DMA_BYTES_PER_FRAME
#define MY_DMA_BUFFER_SIZE_MSIZES MY_DMA_BUFFER_SIZE_BYTES / MY_DMA_BYTES_PER_MSIZE

#define SAMPLE_RATE AUDIO_FREQUENCY_32K


enum {
    BUFFER_STATUS_LOWER_HALF_FULL,
    BUFFER_STATUS_UPPER_HALF_FULL,
    BUFFER_STATUS_IDLE
};
typedef uint8_t BufferStatusMessage_t ;

void Audio_Init(uint32_t frequency, uint8_t volume, uint8_t *TX_buffer, uint8_t *RX_buffer1, uint8_t *RX_buffer2);

void RX_Full(int16_t * sampleBuffer_L, int16_t * sampleBuffer_R, uint32_t num_samples);
void TX_Full(int16_t * sampleBuffer_L, int16_t * sampleBuffer_R, uint32_t num_samples);

void RX_LowerHalf(int16_t * sampleBuffer_L, int16_t * sampleBuffer_R, uint32_t num_samples);
void RX_UpperHalf(int16_t * sampleBuffer_L, int16_t * sampleBuffer_R, uint32_t num_samples);
void TX_LowerHalf(int16_t * sampleBuffer_L, int16_t * sampleBuffer_R, uint32_t num_samples);
void TX_UpperHalf(int16_t * sampleBuffer_L, int16_t * sampleBuffer_R, uint32_t num_samples);


void AUDIO_IN_TransferComplete_CallBack(void);
void AUDIO_IN_HalfTransfer_CallBack(void);
void AUDIO_OUT_Error_CallBack(void);

#endif //F7_LOOPBACK_AUDIO_INIT_H

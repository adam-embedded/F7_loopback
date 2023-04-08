//
// Created by adam slaymark on 28/03/2023.
//
// All credit to: Institute of Science and Technology Austria (IST Austria)
// https://github.com/mgabriel-lt/ap-demodulation
//
// Matlab code converted to C with ARM CMSIS DSP
//
// Arrays have to be allocated at compile time rather than dynamically, this is to increase stability
//

#include "envelope.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#include "audio_init.h"

#define SAMPLES (BUFFER_SIZE/2)

typedef struct {
    int32_t Fs;
    int32_t Fc;
    float32_t Et;
    float32_t Ni;
    uint32_t Ns;
    uint32_t Nx;
    uint8_t D;

} par;

static arm_rfft_fast_instance_f32 J;

void envelope_alloc(){
    arm_rfft_fast_init_f32(&J, BUFFER_SIZE/2);
}

void envelope(int16_t* S, float32_t *m_out){
    //arm_cfft_f32(&arm_cfft_sR_f32_len512);
    //puts("inside envelope");
    par Par = {
            .Fs = SAMPLE_RATE,
            .Fc = 70,
            .Et = powf(10,-5),
            .Ni = powf(10,3),
            .Ns = SAMPLES,
            .Nx = SAMPLES,
            .D = 1,
    };
    float32_t s_abs[SAMPLES];

//    for (int i = 0; i < SAMPLES; i++){
//        printf("%d\n",S[i]);
//    }

    for (int i = 0; i < SAMPLES; i++){
        s_abs[i] = abs(S[i]);
        //printf("%f, ",s_abs[i]);
    }

    uint16_t max_s;
    uint32_t max_index;
    arm_max_q15(&s_abs,SAMPLES,&max_s,&max_index); //maximum of the absolute-value signal

    //puts("through max");
    //printf("MAX: %d\n",max_s);

    for (int i = 0; i < SAMPLES; i++){ //Scaled absolute signal
        s_abs[i] = s_abs[i] / max_s;
        //printf("%f, ",s_abs[i]);
    }

    uint16_t fL = 1 + (uint16_t) ceil(Par.Fc / Par.Fs / Par.Nx);
    uint16_t fR = Par.Nx - fL + 2;

    // Define buffer for samples to work on, storage size should be the max samples
    uint16_t i_cutoff[SAMPLES];

    uint16_t cutoff_count = 0;
    for (uint16_t i = fL; i < fR; i++){
        i_cutoff[cutoff_count] = i;
        cutoff_count++;
    }

    // Initialise infesiability error variables
    float32_t Etol;
    if (Par.Et > 0.0){
        Etol = powf(2, (Par.Et / max_s));
    } else {
      Etol = Par.Et;
    }

    float32_t E = 0;
    for (int i = 0; i < SAMPLES; i++){
        E = E + powf(2, s_abs[i]);
    }
    E = E / Par.Nx;

    uint16_t iter_m = 1;
    uint8_t nim = 0;
    //float32_t m_out[SAMPLES];

    //uint16_t iter_e = 1;
    //uint8_t nie = 0;
    //float32_t e_out;

    static uint32_t iter = 0;
    float32_t m[SAMPLES];

    //Copy absolute values to array to work on
    memcpy(m,s_abs,sizeof(s_abs));

    uint32_t ix[SAMPLES];
    for (int i =0; i < SAMPLES; i++){
        ix[i] = i;
    }

    //puts("before while");
    while (E > Etol && iter < Par.Ni){
        //puts("entered while");
        iter++;

        float32_t a[SAMPLES];
        float32_t a_result[SAMPLES*2];
        memcpy(a,m, sizeof(m));

        //for (int i =0; i < SAMPLES; i++){
            //printf("%f",a[i]);
        //}


        //puts("before forward fft");

        // Forward FFT
        arm_rfft_fast_f32(&J, a, a_result, 0);

        //puts("after forward fft");

        a_result[(i_cutoff[0]*2)] = 0;

        // Reverse FFT
        arm_cfft_f32(&arm_cfft_sR_f32_len512, a_result, 1, 1);

        arm_cmplx_mag_f32(a_result, a, SAMPLES);

        memcpy(m, a, sizeof(a));

        for (int i=0;i < SAMPLES; i++){
            /* Projection onto Cd */
            if (m[i] < s_abs[i]) m[i] = s_abs[i];

            /* Infeasibility error */
            float32_t aux = (m[i]-a[i]);
            E = E + (aux * aux);
        }
        E = E / Par.Nx;

        //printf("PAR.NS: %lu\n", Par.Ns);
        //printf("iter_m: %d\n", iter_m);

        /* Output (modulator) */
        //TODO - error here, there is no met criterion for submitting the output
        if ( iter_m <= nim && (iter == 0 || (E <= Etol && nim == 1 && 0 == Par.Ni)) )
        {
            //m_out(1+(iter_m-1)*ns:iter_m*ns) = m(abs(ix));

            uint32_t i_aux = (iter_m-1)*(Par.Ns);

            for (int i=0; i<(Par.Ns); i++) {
                m_out[i + i_aux] = m[ix[i]] * max_s;
            }
            iter_m++; // = iter_m + 1;
        }

    }
    //puts("after While");
    iter = 0;


}
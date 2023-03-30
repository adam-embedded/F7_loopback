/******************************************************************************
*  File Generated on 29-Mar-2023 
*
*  Coefficient Header file for High Pass Filter
*
*     Sample Frequency: 32000 
*     Cutoff Frequency: 6000 
*     Filter Order: 8 
*
******************************************************************************/

#if !defined(INC_HIGHPASS_COEF_6000HZ_H)
#define INC_HIGHPASS_COEF_6000HZ_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
    Required Header Files
 *============================================================================*/
#include <stdint.h>
#include <arm_math.h>


/*============================================================================*
    Program Start
 *============================================================================*/

#define IIR_ORDER      8
#define IIR_NUMSTAGES (IIR_ORDER/2)


static float32_t highpass_biquad_state[IIR_ORDER];
const float32_t highpass_biquad_coeffs[5*IIR_NUMSTAGES] = 
{
        0.037949285182540,
        -0.075898570365079,
        0.037949285182540,
        0.401529743561799,
        -0.049247784468309,
        1.000000000000000,
        -2.000000000000000,
        1.000000000000000,
        0.432856290508741,
        -0.131107996584992,
        1.000000000000000,
        -2.000000000000000,
        1.000000000000000,
        0.505766864915881,
        -0.321632509121438,
        1.000000000000000,
        -2.000000000000000,
        1.000000000000000,
        0.648484116439535,
        -0.694570659700952,
};

arm_biquad_cascade_df2T_instance_f32 const highPass = 
{
  IIR_ORDER/2,
  highpass_biquad_state,
  highpass_biquad_coeffs
};


#ifdef __cplusplus
}
#endif

#endif /* !defined(INC_HIGHPASS_COEF_6000HZ_H)*/

/*============================================================================*
    End Of File
 *============================================================================*/

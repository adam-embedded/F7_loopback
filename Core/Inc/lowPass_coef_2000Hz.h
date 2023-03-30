/******************************************************************************
*  File Generated on 29-Mar-2023 
*
*  Coefficient Header file for Low Pass Filter
*
*     Sample Frequency: 32000 
*     Cutoff Frequency: 2000 
*     Filter Order: 8 
*
******************************************************************************/

#if !defined(INC_LOWPASS_COEF_2000HZ_H)
#define INC_LOWPASS_COEF_2000HZ_H

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


static float32_t lowpass_biquad_state[IIR_ORDER];
const float32_t lowpass_biquad_coeffs[5*IIR_NUMSTAGES] = 
{
        0.000000888199322,
        0.000001776398644,
        0.000000888199322,
        1.343502062906177,
        -0.454196153966386,
        1.000000000000000,
        2.000000000000000,
        1.000000000000000,
        1.401739933120037,
        -0.517232370447512,
        1.000000000000000,
        2.000000000000000,
        1.000000000000000,
        1.523789873410176,
        -0.649338273863707,
        1.000000000000000,
        2.000000000000000,
        1.000000000000000,
        1.719392914169195,
        -0.861057479534746,
};

arm_biquad_cascade_df2T_instance_f32 const lowPass = 
{
  IIR_ORDER/2,
  lowpass_biquad_state,
  lowpass_biquad_coeffs
};


#ifdef __cplusplus
}
#endif

#endif /* !defined(INC_LOWPASS_COEF_2000HZ_H)*/

/*============================================================================*
    End Of File
 *============================================================================*/

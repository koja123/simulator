//
// Link metric
//
#ifndef PACKET_LINK_METRIC_H_
#define PACKET_LINK_METRIC_H_

#include "config.h" //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup link_metric Core :: Link metric
 * @{
 */

/** Type of link metric value
 */
#define LINK_METRIC_TYPE__NONE      (-1)    ///< metric type is NONE
#define LINK_METRIC_TYPE__ETX       (10)    ///< metric type is ETX
#define LINK_METRIC_TYPE__STATIC    (30)    ///< metric type is STATIC
#define LINK_METRIC_TYPE__TEST      (60)    ///< metric type is TEST

/** Link metric value
 */
typedef double   nu_link_metric_t;

/*
 * link metric value parameters
 */
#define MINIMUM_METRIC              (16)         ///< min metric
#define MAXIMUM_METRIC              (1015808)    ///< max metric
#define DEFAULT_METRIC              (4096)       ///< default metric
#define UNDEF_METRIC                (0x7fffffff) ///< undef metric

/** Link metric type code defined by 2bits used in TLV.
 */
#define LM_TYPE_CODE__L_IN_N_OUT    (0) ///< metric type code is L_IN_N_OUT
#define LM_TYPE_CODE__L_IN          (1) ///< metric type code is L_IN
#define LM_TYPE_CODE__N_OUT         (2) ///< metric type code is N_OUT
#define LM_TYPE_CODE__N_IN          (3) ///< metric type code is N_IN

struct tuple_i;

/** Gets type ext code of link metric TLV.
 *
 * @param code
 *	encoded code
 * @return direction
 */
PUBLIC_INLINE int
nu_link_metric_direction(uint8_t code)
{
    return code >> 6;
}

/** Gets link_metric_type code
 *
 * @param code
 *	encoded code
 * @return decoded code
 */
PUBLIC_INLINE int
nu_link_metric_type(uint8_t code)
{
    return code & 0x3f;
}

/** Gets link_metric_type_ext code
 *
 * @param type
 * @param direction
 * @return code
 */
PUBLIC_INLINE uint8_t
nu_link_metric_type_ext(int type, int direction)
{
    return (uint8_t)(type | (direction << 6));
}

// encode / decode link metric /////////////////////////////////

/** Scale factor for encoding link metric value */
#define LINK_METRIC_SCALE_FACTOR    (16)

/** Converts a link metric to a mantissa/exponent product.
 *
 * @param t
 * @return the 8-bit mantissa/exponent product
 */
PUBLIC_INLINE uint8_t
nu_link_metric_pack(nu_link_metric_t t)
{
    int a;
    int b = 0;
    while (b <= 16 && t / LINK_METRIC_SCALE_FACTOR >= pow((double)2, (double)b))
        ++b;

    --b;
    if (b < 0) {
        a = 0;
        b = 0;
    } else if (b > 15) {
        a = 15;
        b = 15;
    } else {
#if defined(_WIN32) || defined(_WIN64)                                                            //ScenSim-Port://
        a = (int)(16 * ((double)t / (LINK_METRIC_SCALE_FACTOR * (double)pow((double)2, b)) - 1)); //ScenSim-Port://
#else                                                                                             //ScenSim-Port://
        a = (int)(16 * ((double)t / (LINK_METRIC_SCALE_FACTOR * (double)pow(2, b)) - 1));
#endif                                                                                            //ScenSim-Port://
        while (a >= 16) {
            a -= 16;
            ++b;
        }
    }
    return (uint8_t)(b * 16 + a);
}

/** Converts a mantissa/exponent 8bit value back to link metric.
 *
 * @param me	8 bit mantissa/exponen value
 * @return the decoded value
 * @see RFC5497.
 */
PUBLIC_INLINE nu_link_metric_t
nu_link_metric_unpack(uint8_t me)
{
    int b = me >> 4;
    int a = me - b * 16;
#if defined(_WIN32) || defined(_WIN64)                                                                      //ScenSim-Port://
    return (nu_link_metric_t)(LINK_METRIC_SCALE_FACTOR * (1 + (double)a / 16) * (double)pow((double)2, b)); //ScenSim-Port://
#else                                                                                                       //ScenSim-Port://
    return (nu_link_metric_t)(LINK_METRIC_SCALE_FACTOR * (1 + (double)a / 16) * (double)pow(2, b));
#endif                                                                                                      //ScenSim-Port://
}

/** @} */

}//namespace// //ScenSim-Port://

#endif

#ifndef OLSRV2_ETX_H_
#define OLSRV2_ETX_H_

#include "config.h" //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup  olsrv2-etx OLSRv2-ETX :: ETX
 * @{
 */

// ETX Parameters
#define DEFAULT_ETX_MEMORY_LENGTH       (32)                   ///< ETX: LIFO length
#define DEFAULT_ETX_METRIC_INTERVAL     (1)                    ///< ETX: Metric interval
#define ETX_SEQNUM_RESTART_DETECTION    (256)                  ///< ETX: Seqnum restart detection
#define ETX_HELLO_TIMEOUT_FACTOR        (1.5)                  ///< ETX: Timeout factor
#define ETX_PERFECT_METRIC              (DEFAULT_METRIC * 0.1) ///< ETX: Prefect metric

//
#define UNDEF_INTERVAL                  (-1)    ///< UNDEFINED INTERVAL
#define UNDEF_ETX                       (-1.0)  ///< UNDEFINED ETX
#define UNDEF_SEQNUM                    (-1)    ///< UNDEFINED SEQNUM

PUBLIC void etx_hello_timeout_event(nu_event_t*);
PUBLIC void etx_calc_metric_event(nu_event_t*);

#define ETX_SCALE_FACTOR    (1.0 / 1024)    ///< ETX scale factor


/** Converts a double to a mantissa/exponent product as described in RFC5497.
 *
 * @param t	the time to process
 * @return the 8-bit mantissa/exponent product
 */
PUBLIC_INLINE uint8_t
etx_pack(double t)
{
    int a;
    int b = 0;
    while (b <= 32 && t / ETX_SCALE_FACTOR >= pow((double)2, (double)b))
        ++b;

    --b;
    if (b < 0) {
        a = 0;
        b = 0;
    } else if (b > 31) {
        a = 7;
        b = 31;
    } else {
#if defined(_WIN32) || defined(_WIN64)                                                   //ScenSim-Port://
        a = (int)(8 * ((double)t / (ETX_SCALE_FACTOR * (double)pow((double)2, b)) - 1)); //ScenSim-Port://
#else                                                                                    //ScenSim-Port://
        a = (int)(8 * ((double)t / (ETX_SCALE_FACTOR * (double)pow(2, b)) - 1));
#endif                                                                                   //ScenSim-Port://
        while (a >= 8) {
            a -= 8;
            ++b;
        }
    }
    return (uint8_t)(b * 8 + a);
}

/** Converts a mantissa/exponent 8bit value back to double.
 *
 * @param me	8 bit mantissa/exponen value
 * @return the decoded value
 * @see RFC5497.
 */
PUBLIC_INLINE double
etx_unpack(uint8_t me)
{
    int b = me >> 3;
    int a = me - b * 8;
#if defined(_WIN32) || defined(_WIN64)                                                   //ScenSim-Port://
    return (double)(ETX_SCALE_FACTOR * (1 + (double)a / 8) * (double)pow((double)2, b)); //ScenSim-Port://
#else                                                                                    //ScenSim-Port://
    return (double)(ETX_SCALE_FACTOR * (1 + (double)a / 8) * (double)pow(2, b));
#endif                                                                                   //ScenSim-Port://
}

/** @} */

}//namespace// //ScenSim-Port://

#endif

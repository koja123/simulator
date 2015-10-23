#ifndef NU_CORE_TIME_H_
#define NU_CORE_TIME_H_

#include "config.h" //ScenSim-Port://
#include <math.h>

namespace NuOLSRv2Port { //ScenSim-Port://

/***********************************************************//***
 * @defgroup nu_time Core :: Time functions
 * @{
 */

typedef struct timeval   nu_time_t;
extern void NuOLSRv2ProtocolScheduleEvent(const nu_time_t& time); //ScenSim-Port://

#define VTIME_SCALE_FACTOR    (1.0 / 1024)
#define MICRO                 (1000000)

/** Compares times.
 *
 * @param a
 * @param b
 * @retval 0  if a == b
 * @retval <0 if a < b
 * @retval >0 if a > b
 */
PUBLIC_INLINE int
nu_time_cmp(const nu_time_t a, const nu_time_t b)
{
    if (a.tv_sec != b.tv_sec)
        return a.tv_sec - b.tv_sec;
    else
        return a.tv_usec - b.tv_usec;
}

/** Sets time to zero.
 *
 * @param[out] time
 */
PUBLIC_INLINE void
nu_time_set_zero(nu_time_t* time)
{
    time->tv_sec  = 0;
    time->tv_usec = 0;
}

#ifndef nuOLSRv2_SIMULATOR

/** Sets the current time to the time
 *
 * @param[out] time
 */
PUBLIC_INLINE void
nu_time_set_now(nu_time_t* time)
{
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    assert(false);                     //ScenSim-Port://
    abort();                           //ScenSim-Port://
#else                                  //ScenSim-Port://
    gettimeofday(time, NULL);
#endif                                 //ScenSim-Port://
}

#endif

/** Sets the specified time[sec] to the time.
 *
 * @param time
 * @param sec
 */
PUBLIC_INLINE void
nu_time_set_f(nu_time_t* time, const double sec)
{
    time->tv_sec  = (long)sec;
    time->tv_usec = (long)(sec * MICRO - ((double)time->tv_sec) * MICRO);
    assert(time->tv_sec >= 0);
    assert(time->tv_usec >= 0);
}

/** Adds the specified time[src] to the time.
 *
 * @param time
 * @param update
 */
PUBLIC_INLINE void
nu_time_add_sec(nu_time_t* time, const int update)
{
    time->tv_sec += update;
    assert(time->tv_sec >= 0);
    assert(time->tv_usec >= 0);
}

/** Adds the specified time[src] to the time.
 *
 * @param time
 * @param update
 */
PUBLIC_INLINE void
nu_time_add_f(nu_time_t* time, const double update)
{
    const long update_sec  = (long)update;
    const long update_usec = (long)(update * MICRO - ((double)update_sec) * MICRO);

    time->tv_usec += update_usec;
    time->tv_sec  += update_sec + time->tv_usec / MICRO;
    time->tv_usec %= MICRO;
    assert(time->tv_sec >= 0);
    assert(time->tv_usec >= 0);
}

/** Converts the time to double value.
 *
 * @param time
 * @return the time
 */
PUBLIC_INLINE double
nu_time_get_f(const nu_time_t* time)
{
    return ((double)time->tv_sec) + ((double)time->tv_usec) / MICRO;
}

/** Subtract the time from the time.
 *
 * @param t1
 * @param t2
 * @return t1 - t2
 */
PUBLIC_INLINE double
nu_time_sub(const nu_time_t t1, const nu_time_t t2)
{
    return ((double)t1.tv_sec) - ((double)t2.tv_sec)
           + (((double)t1.tv_usec) - ((double)t2.tv_usec)) / MICRO;
}

/** Returns the bigger time.
 *
 * @param a
 * @param b
 * @retval a if a >= b
 * @retval b if a < b
 */
PUBLIC_INLINE nu_time_t
nu_time_max(const nu_time_t a, const nu_time_t b)
{
    if (a.tv_sec == b.tv_sec) {
        if (a.tv_usec >= b.tv_sec)
            return a;
        else
            return b;
    } else {
        if (a.tv_sec >= b.tv_sec)
            return a;
        else
            return b;
    }
}

/** Returns the smaller time.
 *
 * @param a
 * @param b
 * @retval a if a <= b
 * @retval b if a > b
 */
PUBLIC_INLINE nu_time_t
nu_time_min(const nu_time_t a, const nu_time_t b)
{
    if (a.tv_sec == b.tv_sec) {
        if (a.tv_usec >= b.tv_sec)
            return b;
        else
            return a;
    } else {
        if (a.tv_sec >= b.tv_sec)
            return b;
        else
            return a;
    }
}

/** Converts a double to a mantissa/exponent product as described in RFC5497.
 *
 * @param t	the time to process
 * @return the 8-bit mantissa/exponent product
 */
PUBLIC_INLINE uint8_t
nu_time_pack(double t)
{
    int a;
    int b = 0;
    while (b <= 32 && t / VTIME_SCALE_FACTOR >= pow((double)2, (double)b))
        ++b;

    --b;
    if (b < 0) {
        a = 0;
        b = 0;
    } else if (b > 31) {
        a = 7;
        b = 31;
    } else {
#if defined(_WIN32) || defined(_WIN64)                                                     //ScenSim-Port://
        a = (int)(8 * ((double)t / (VTIME_SCALE_FACTOR * (double)pow((double)2, b)) - 1)); //ScenSim-Port://
#else                                                                                      //ScenSim-Port://
        a = (int)(8 * ((double)t / (VTIME_SCALE_FACTOR * (double)pow(2, b)) - 1));
#endif                                                                                     //ScenSim-Port://
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
nu_time_unpack(uint8_t me)
{
    int b = me >> 3;
    int a = me - b * 8;
#if defined(_WIN32) || defined(_WIN64)                                                     //ScenSim-Port://
    return (double)(VTIME_SCALE_FACTOR * (1 + (double)a / 8) * (double)pow((double)2, b)); //ScenSim-Port://
#else                                                                                      //ScenSim-Port://
    return (double)(VTIME_SCALE_FACTOR * (1 + (double)a / 8) * (double)pow(2, b));
#endif                                                                                     //ScenSim-Port://
}

/** @} */

}//namespace// //ScenSim-Port://

#endif

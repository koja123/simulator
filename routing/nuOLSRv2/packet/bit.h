#ifndef NU_PACKET_BIT_H_
#define NU_PACKET_BIT_H_


#define bit_set(v, bit)         \
    do { (v) |= (1 << (bit)); } \
    while (0)
#define bit_clr(v, bit)        \
    do { (v) &= ~(1 << bit); } \
    while (0)

#define bit_is_set(v, bit)    (((v) & (1 << (bit))) != 0)
#define bit_is_clr(v, bit)    (((v) & (1 << (bit))) == 0)


#endif

#ifndef NU_PACKET_MSGTLV_H_
#define NU_PACKET_MSGTLV_H_

#define msgtlv_create_validity_time(t) \
    nu_tlv_create_u8(MSG_TLV__VALIDITY_TIME, 0, nu_time_pack(t))
#define msgtlv_create_interval_time(t) \
    nu_tlv_create_u8(MSG_TLV__INTERVAL_TIME, 0, nu_time_pack(t))
#define msgtlv_create_mpr_willing(w) \
    nu_tlv_create_u8(MSG_TLV__MPR_WILLING, 0, w)
#define msgtlv_create_cont_seqnum(ext, num) \
    nu_tlv_create_u16(MSG_TLV__CONT_SEQNUM, ext, num)

#endif

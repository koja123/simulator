#ifndef NU_PACKET_ADDRTLV_H_
#define NU_PACKET_ADDRTLV_H_

#define addrtlv_create_link_status(s) \
    nu_tlv_create_u8(ADDR_TLV__LINK_STATUS, 0, s)
#define addrtlv_create_local_if(t) \
    nu_tlv_create_u8(ADDR_TLV__LOCAL_IF, 0, t)
#define addrtlv_create_other_neighb(t) \
    nu_tlv_create_u8(ADDR_TLV__OTHER_NEIGHB, 0, t)
#if 0                                        //ScenSim-Port://
#define addrtlv_create_mpr(t) \
    nu_tlv_create_novalue(ADDR_TLV__MPR, 0)
#define addrtlv_create_flooding_mpr(t) \
    nu_tlv_create_novalue(ADDR_TLV__FMPR, 0)
#define addrtlv_create_routing_mpr(t) \
    nu_tlv_create_novalue(ADDR_TLV__RMPR, 0)
#else                                        //ScenSim-Port://
#define addrtlv_create_mpr() \
    nu_tlv_create_novalue(ADDR_TLV__MPR, 0)  //ScenSim-Port://
#define addrtlv_create_flooding_mpr() \
    nu_tlv_create_novalue(ADDR_TLV__FMPR, 0) //ScenSim-Port://
#define addrtlv_create_routing_mpr() \
    nu_tlv_create_novalue(ADDR_TLV__RMPR, 0) //ScenSim-Port://
#endif                                       //ScenSim-Port://
#define addrtlv_create_gateway(d) \
    nu_tlv_create_u8(ADDR_TLV__GATEWAY, 0, d)
#define addrtlv_create_nbr_addr_type(t) \
    nu_tlv_create_u8(ADDR_TLV__NBR_ADDR_TYPE, 0, t)
#define addrtlv_create_link_metric(ext, value) \
    nu_tlv_create_u8(ADDR_TLV__LINK_METRIC, ext, nu_link_metric_pack(value))
#define addrtlv_create_r_etx(value) \
    nu_tlv_create_u8(ADDR_TLV__R_ETX, 0, etx_pack(value))

#endif

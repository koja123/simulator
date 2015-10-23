#include "config.h"

#include "core/core.h"
#include "packet/constant.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_packet
 * @{
 */

/* *INDENT-OFF* */

/** Link Metric Tlv Type
 */
PUBLIC manet_constant_t manet_tlv_link_metric[1]  = {
    { 0xff, NULL, NULL }
};

/** General Time Tlv Type
 */
PUBLIC manet_constant_t manet_tlv_general_time[1] = {
    { 0xff, NULL, NULL }
};

/** Packet Tlv Type
 */
PUBLIC manet_constant_t manet_pkt_tlv[1] = {
    { 0xff, NULL, NULL },
};

/** Message Type
 */
PUBLIC manet_constant_t manet_msg_type[3] = {
    { MSG_TYPE__HELLO, "hello", NULL },
    { MSG_TYPE__TC,    "tc",    NULL },
    { 0xff, NULL, NULL },
};

/** Message Tlv Type
 */
PUBLIC manet_constant_t manet_msg_tlv[6] = {
    { MSG_TLV__VALIDITY_TIME,   "validity_time", manet_tlv_general_time },
    { MSG_TLV__INTERVAL_TIME,   "interval_time", manet_tlv_general_time },
    { MSG_TLV__MPR_WILLING,     "mpr_willing",   NULL },
    { MSG_TLV__CONT_SEQNUM,     "cont_seqnum",   NULL },
    { MSG_TLV__LINK_METRIC_EXT, "link_metric_ext", NULL },
    { 0xff, NULL, NULL },
};

/** ADDR_TLV__LINK_STATUS's Value
 */
PUBLIC manet_constant_t manet_link_status[6] = {
    { LINK_STATUS__LOST,      "lost",      NULL },
    { LINK_STATUS__SYMMETRIC, "symmetric", NULL },
    { LINK_STATUS__HEARD,     "heard",     NULL },
    { LINK_STATUS__PENDING,   "pending",   NULL },
    { LINK_STATUS__UNKNOWN,   "unknown",   NULL },
    { 0xff, NULL, NULL },
};

/** ADDR_TLV__LOCAL_IF's Value
 */
PUBLIC manet_constant_t manet_local_if[4] = {
    { LOCAL_IF__THIS_IF,   "this_if",   NULL },
    { LOCAL_IF__OTHER_IF,  "other_if",  NULL },
    { LOCAL_IF__UNSPEC_IF, "unspec_if", NULL },
    { 0xff, NULL, NULL },
};

/** ADDR_TLV__OTHER_NEIGHB's Value
 */
PUBLIC manet_constant_t manet_other_neighb[3] = {
    { OTHER_NEIGHB__LOST,      "lost",      NULL },
    { OTHER_NEIGHB__SYMMETRIC, "symmetric", NULL },
    { 0xff, NULL, NULL },
};

/** NBR_ADDR_TYPE's Value
 */
PUBLIC manet_constant_t manet_nbr_addr_type[4] = {
    { NBR_ADDR_TYPE__ORIGINATOR,    "orig",          NULL },
    { NBR_ADDR_TYPE__ROUTABLE,      "routable",      NULL },
    { NBR_ADDR_TYPE__ROUTABLE_ORIG, "routable_orig", NULL },
    { 0xff, NULL, NULL },
};

/** Address Tlv Type
 */
PUBLIC manet_constant_t manet_addr_tlv[11] = {
    { ADDR_TLV__LINK_STATUS,   "link_status",   manet_link_status   },
    { ADDR_TLV__LOCAL_IF,      "local_if",      manet_local_if      },
    { ADDR_TLV__OTHER_NEIGHB,  "other_neighb",  manet_other_neighb  },
    { ADDR_TLV__MPR,           "mpr",           NULL                },
    { ADDR_TLV__FMPR,          "fmpr",          NULL                },
    { ADDR_TLV__RMPR,          "rmpr",          NULL                },
    { ADDR_TLV__NBR_ADDR_TYPE, "nbr_addr_type", manet_nbr_addr_type },
    { ADDR_TLV__GATEWAY,       "gateway",       NULL                },
    { ADDR_TLV__LINK_METRIC,   "link_metric",   manet_tlv_link_metric },
    { ADDR_TLV__R_ETX,         "r_etx",         manet_tlv_link_metric },
    { 0xff, NULL, NULL },
};

/* *INDENT-ON* */

/** @} */

}//namespace// //ScenSim-Port://

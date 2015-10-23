#ifndef NU_PACKET_CONSTANT_H_
#define NU_PACKET_CONSTANT_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/**
 * MANET Constant
 */
typedef struct manet_constant {
    uint8_t                code;  ///< code
    const char*                  name;  ///< name
    const struct manet_constant* value; ///< values
} manet_constant_t;

/** Packet Tlv Type
 */
#ifndef nuOLSRv2_ALL_STATIC
EXTERN manet_constant_t manet_pkt_tlv[1];
#endif

/** Message Type
 */
#define MSG_TYPE__HELLO    (5)
#define MSG_TYPE__TC       (6)
#ifndef nuOLSRv2_ALL_STATIC
EXTERN manet_constant_t manet_msg_type[3];
#endif

/** Message Tlv Type
 */
#define MSG_TLV__VALIDITY_TIME      (35)
#define MSG_TLV__INTERVAL_TIME      (30)
#define MSG_TLV__MPR_WILLING        (4)
#define MSG_TLV__CONT_SEQNUM        (6)
#define MSG_TLV__LINK_METRIC_EXT    (10)
#ifndef nuOLSRv2_ALL_STATIC
EXTERN manet_constant_t manet_msg_tlv[6];
#endif

/** ADDR_TLV__LINK_STATUS's Value
 */
#define LINK_STATUS__LOST         (0)
#define LINK_STATUS__SYMMETRIC    (1)
#define LINK_STATUS__HEARD        (2)
#define LINK_STATUS__PENDING      (3)
#define LINK_STATUS__UNKNOWN      (5)
#ifndef nuOLSRv2_ALL_STATIC
EXTERN manet_constant_t manet_link_status[6];
#endif

/** ADDR_TLV__LOCAL_IF's Value
 */
#define LOCAL_IF__THIS_IF      (0)
#define LOCAL_IF__OTHER_IF     (1)
#define LOCAL_IF__UNSPEC_IF    (2)
#ifndef nuOLSRv2_ALL_STATIC
EXTERN manet_constant_t manet_local_if[4];
#endif

/** ADDR_TLV__OTHER_NEIGHB's Value
 */
#define OTHER_NEIGHB__LOST         (0)
#define OTHER_NEIGHB__SYMMETRIC    (1)
#ifndef nuOLSRv2_ALL_STATIC
EXTERN manet_constant_t manet_other_neighb[3];
#endif

/** NBR_ADDR_TYPE's Value
 */
#define NBR_ADDR_TYPE__ORIGINATOR       (1)
#define NBR_ADDR_TYPE__ROUTABLE         (2)
#define NBR_ADDR_TYPE__ROUTABLE_ORIG    (3)
#ifndef nuOLSRv2_ALL_STATIC
EXTERN manet_constant_t manet_nbr_addr_type[4];
#endif

/** Address Tlv Type
 */
#define ADDR_TLV__LOCAL_IF         (1)
#define ADDR_TLV__LINK_STATUS      (2)
#define ADDR_TLV__OTHER_NEIGHB     (3)
#define ADDR_TLV__MPR              (5)
#define ADDR_TLV__GATEWAY          (7)
#define ADDR_TLV__NBR_ADDR_TYPE    (8)
#define ADDR_TLV__LINK_METRIC      (10)
#define ADDR_TLV__RMPR             (11)
#define ADDR_TLV__FMPR             (12)
#define ADDR_TLV__R_ETX            (13)
#ifndef nuOLSRv2_ALL_STATIC
EXTERN manet_constant_t manet_addr_tlv[11];
#endif

// CONT_SEQNUM's  type_ext
#define CONT_SEQNUM_EXT__COMPLETE      (0)
#define CONT_SEQNUM_EXT__INCOMPLETE    (1)

#ifndef nuOLSRv2_ALL_STATIC
EXTERN manet_constant_t manet_tlv_general_time[1];
EXTERN manet_constant_t manet_tlv_link_metric[1];
#endif

}//namespace// //ScenSim-Port://

#endif

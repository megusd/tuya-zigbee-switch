#ifndef ZCL_FAN_CONTROL_H
#define ZCL_FAN_CONTROL_H

#pragma pack(push, 1)
#include "zcl_include.h"
#pragma pack(pop)

#define ZCL_CLUSTER_HVAC_FAN_CONTROL    0x0202

/**
 * Register the Fan Control cluster (0x0202) via the Telink SDK's generic
 * zcl_registerCluster() path.  The TLSR8258 SDK does not provide a dedicated
 * fan-control module, so we use the generic custom-cluster registration.
 */
status_t zcl_fan_control_register(u8 endpoint, u16 manuCode, u8 attrNum,
                                  const zclAttrInfo_t attrTbl[],
                                  cluster_forAppCb_t cb);

#endif /* ZCL_FAN_CONTROL_H */

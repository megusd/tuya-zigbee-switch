#pragma pack(push, 1)
#include "zcl_include.h"
#pragma pack(pop)

_CODE_ZCL_ status_t zcl_color_control_register(u8 endpoint, u16 manuCode,
                                                u8 attrNum,
                                                const zclAttrInfo_t attrTbl[],
                                                cluster_forAppCb_t cb) {
    /* Use the generic cluster registration path — safe across all SDK versions.
     * Commands are dispatched via the cmd_callback registered in zigbee_zcl.c. */
    return zcl_registerCluster(endpoint, ZCL_CLUSTER_LIGHTING_COLOR_CONTROL,
                               manuCode, attrNum, attrTbl, NULL, cb);
}

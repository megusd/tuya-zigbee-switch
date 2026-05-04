#ifndef ZCL_COLOR_CONTROL_H
#define ZCL_COLOR_CONTROL_H

#pragma pack(push, 1)
#include "zcl_include.h"
#pragma pack(pop)

/**
 * Register the Color Control cluster (0x0300) on the given endpoint using the
 * generic Telink SDK cluster registration path.
 */
status_t zcl_color_control_register(u8 endpoint, u16 manuCode, u8 attrNum,
                                    const zclAttrInfo_t attrTbl[],
                                    cluster_forAppCb_t cb);

#endif /* ZCL_COLOR_CONTROL_H */

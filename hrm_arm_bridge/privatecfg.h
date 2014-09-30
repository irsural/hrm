#ifndef PRIVATECFGH
#define PRIVATECFGH

#include <irsdefs.h>

#include <irsfinal.h>

#define IP_0 192
#define IP_1 168
#define IP_2 0
#define IP_3 203

#define HRM_MASK_0 255
#define HRM_MASK_1 255
#define HRM_MASK_2 255
#define HRM_MASK_3 0

#define HRM_GATEWAY_0 0
#define HRM_GATEWAY_1 0
#define HRM_GATEWAY_2 0
#define HRM_GATEWAY_3 0

#define HRM_DHCP_ON 0

#define MAKE_IP(IP_0, IP_1, IP_2, IP_3) #IP_0 "." #IP_1 "." #IP_2 "." #IP_3
#define IP_STR MAKE_IP(IP_0, IP_1, IP_2, IP_3)

#define R2R_ON_X2 1

#endif //PRIVATECFGH

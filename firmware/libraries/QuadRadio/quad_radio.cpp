#include "quad_radio.h"

int parse_msg(uint8_t* msg_bytes, rfInfo_t &g){
    rfInfo_t info;
    memcpy((void *) &info, (void *) msg_bytes, sizeof(rfInfo_t));
    if (validate(info)) {
        g = info;
        return GOOD_PACKET;
    } else {
        return BAD_PACKET;
    }
}

bool validate(rfInfo_t info) {
    return compute_parity(info) == info.parity && info.magic == MAGIC_NUMBER;
}

uint8_t* get_msg(rfInfo_t info)
{
	info.parity = compute_parity(info);
    uint8_t* info_buffer = (uint8_t*)malloc(sizeof(rfInfo_t));
    memcpy(info_buffer, &info, sizeof(rfInfo_t));
	return info_buffer;
}

uint8_t compute_parity(rfInfo info) {
    return info.magic^info.armed^info.throttle^info.yaw^info.pitch^info.roll;
}
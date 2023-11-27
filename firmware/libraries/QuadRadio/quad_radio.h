#include <Arduino.h>

#define GOOD_PACKET 1
#define BAD_PACKET 0
#define MAGIC_NUMBER 123

typedef struct rfInfo {
    uint8_t magic;
    uint8_t armed;
    uint16_t throttle;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
    float pid_yaw[3];
    float pid_pitch[3];
    float pid_roll[3];
    uint8_t parity;
    rfInfo() : magic(MAGIC_NUMBER), parity(0) {}
    // rfInfo(uint8_t* input) {
    //     magic = input[0];
    //     armed = input[1];
    //     throttle = input[2];
    //     yaw = input[3];
    //     pitch = input[4];
    //     roll = input[5];
    //     pid_yaw[0] = input[6];
    //     pid_yaw[1] = input[7];
    //     pid_yaw[2] = input[8];
    //     pid_pitch[0] = input[9];
    //     pid_pitch[1] = input[10];
    //     pid_pitch[2] = input[11];
    //     pid_roll[0] = input[12];
    //     pid_roll[1] = input[13];
    //     pid_roll[2] = input[14];
    //     parity = input[15];
    // }
}__attribute__ ((packed)) rfInfo_t;


bool validate(rfInfo_t);
uint8_t compute_parity(rfInfo_t);

uint8_t* get_msg(rfInfo_t);

int parse_msg(uint8_t*, rfInfo_t &);